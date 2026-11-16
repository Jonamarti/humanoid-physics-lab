param([string]$Python = "3.12", [string]$CMake = "")
$ErrorActionPreference = "Stop"
$labRoot = (Resolve-Path (Join-Path $PSScriptRoot "../..")).Path
Set-Location -LiteralPath $labRoot
$env:GIT_LFS_SKIP_SMUDGE = "1"
$env:WANDB_MODE = "disabled"
function Check-Exit { if ($LASTEXITCODE -ne 0) { throw "External command failed: $LASTEXITCODE" } }
function Checkout-Pinned($url,$path,$sha) {
    if (-not (Test-Path -LiteralPath $path)) {
        git clone --filter=blob:none --no-checkout $url $path
        Check-Exit
        git -C $path checkout $sha
        Check-Exit
    }
    $actual = (git -C $path rev-parse HEAD).Trim()
    Check-Exit
    if ($actual -ne $sha) { throw "Unexpected external revision in $path; refusing to overwrite" }
}
New-Item -ItemType Directory -Force out/native,out/external | Out-Null
Checkout-Pinned "https://github.com/NVlabs/ProtoMotions.git" "out/external/protomotions" "13de0cccd06f53ce5c9cf9ec7d1932c7da73610f"
Checkout-Pinned "https://github.com/google-deepmind/mujoco_mpc.git" "out/external/mujoco_mpc" "ff572a21e7c2bf9fda62e1862a758da7e9a8719b"
if (-not (Test-Path out/native/.venv/Scripts/python.exe)) {
    uv venv --python $Python out/native/.venv
    Check-Exit
}
$labPython = Join-Path $labRoot "out/native/.venv/Scripts/python.exe"
uv pip install --python $labPython --index-url https://download.pytorch.org/whl/cpu "torch==2.8.0+cpu"
Check-Exit
uv pip install --python $labPython -c scripts/controller_lab/constraints-windows.txt -e "out/external/protomotions[mujoco]" "mujoco==3.5.0" imageio-ffmpeg pillow
Check-Exit
# Checkpoint only. Do not fetch BONES-SEED motion packs or all Git LFS assets.
git -C out/external/protomotions lfs pull "--include=data/pretrained_models/motion_tracker/anatomic_soma_bones_fsq/inference_last.ckpt" "--exclude="
Check-Exit
$checkpoint = "out/external/protomotions/data/pretrained_models/motion_tracker/anatomic_soma_bones_fsq/inference_last.ckpt"
if ((Get-FileHash -LiteralPath $checkpoint -Algorithm SHA256).Hash.ToLower() -ne "c13e56e85494f8c93fe9eed5b07cadef7281c2eb1188231755f858e1d0f340f0") {
    throw "Checkpoint checksum mismatch"
}
$zip = Join-Path $labRoot "out/native/mujoco-sdk.zip"
if (-not (Test-Path -LiteralPath $zip)) {
    Invoke-WebRequest -Uri "https://github.com/google-deepmind/mujoco/releases/download/3.5.0/mujoco-3.5.0-windows-x86_64.zip" -OutFile $zip
}
if ((Get-FileHash -LiteralPath $zip -Algorithm SHA256).Hash.ToLower() -ne "877d0dfbceac3de90a874c41e0f20c568d104e8ca19de955c0482e3b63832519") {
    throw "SDK checksum mismatch"
}
if (-not (Test-Path out/native/sdk/lib/mujoco.lib)) {
    Expand-Archive -LiteralPath $zip -DestinationPath out/native/sdk
}
if (-not $CMake) {
    $found = Get-Command cmake -ErrorAction SilentlyContinue
    if ($found) { $CMake = $found.Source }
    else {
        $vswhere = "${env:ProgramFiles(x86)}\Microsoft Visual Studio\Installer\vswhere.exe"
        $vsPath = & $vswhere -latest -products "*" -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath
        $CMake = Join-Path $vsPath "Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe"
    }
}
if (-not (Test-Path -LiteralPath $CMake)) { throw "Install Visual Studio C++ Build Tools, or pass -CMake" }
$source = (Join-Path $labRoot "out/external/mujoco_mpc").Replace("\","/")
$sdk = (Join-Path $labRoot "out/native/sdk").Replace("\","/")
& $CMake -S scripts/controller_lab/mjpc -B out/native/mjpc-build -G "Visual Studio 17 2022" -A x64 "-DMJPC_SOURCE:PATH=$source" "-DMUJOCO_SDK:PATH=$sdk"
Check-Exit
& $CMake --build out/native/mjpc-build --target hpl_mjpc --config Release --parallel 2
Check-Exit
if (-not (Test-Path out/controller-comparison/references.pt)) {
    & $labPython scripts/controller_lab/benchmark.py --prepare
    Check-Exit
}
& $labPython scripts/controller_lab/test_lab.py
Check-Exit
