import assert from "node:assert/strict";
import { existsSync } from "node:fs";
import { mkdir, writeFile } from "node:fs/promises";
import path from "node:path";
import { chromium } from "playwright";

/** Smoke the REAL public GitHub Pages build, not a local Vite preview.
 * This verifies browser/runtime availability; it does not certify physical
 * recovery, landing or biomechanical realism.
 */
const URL = "https://jonamarti.github.io/humanoid-physics-lab/";
const expected = process.env.HPL_EXPECTED_DEPLOY_SHA ?? "";
if (!/^[0-9a-f]{40}$/i.test(expected)) {
  throw new Error("HPL_EXPECTED_DEPLOY_SHA must be the deployed main commit SHA");
}
const output = path.resolve("media/deployed-smoke");
await mkdir(output, { recursive: true });
const executablePath = [
  process.env.CHROME_EXECUTABLE ?? "",
  "/usr/bin/google-chrome", "/usr/bin/google-chrome-stable", "/usr/bin/chromium",
].find(p => p && existsSync(p));

const browser = await chromium.launch({
  headless: true,
  ...(executablePath ? { executablePath } : {}),
  args: [
    "--no-sandbox", "--disable-dev-shm-usage",
    "--enable-webgl", "--use-angle=swiftshader", "--enable-unsafe-swiftshader",
  ],
});
const results = [];

async function advanceTo(page, seconds) {
  await page.waitForFunction(t => {
    const s = document.querySelector("#physics-readout")?.textContent ?? "";
    const m = s.match(/t\s*=\s*([\d.]+)\s*s/);
    return m && Number(m[1]) >= t;
  }, seconds, { timeout: 100000, polling: "raf" });
}

async function loadStanding(page) {
  await page.locator("#physics-model").selectOption("standing");
  await page.locator("#physics-load").click();
  await page.waitForFunction(
    () => document.querySelector("#physics-toggle")?.disabled === false,
    undefined, { timeout: 80000 },
  );
  assert.equal(await page.locator("#physics-viewport canvas").count(), 1,
    "MuJoCo/Three.js canvas missing after load");
  const status = await page.locator("#physics-status").innerText();
  assert(!status.includes("No se pudo"), "Standing mode failed: " + status);
}

async function scenario(name, viewport, callback) {
  const context = await browser.newContext({
    viewport, deviceScaleFactor: 1,
    ignoreHTTPSErrors: false,
  });
  const page = await context.newPage();
  const errors = [];
  page.on("pageerror", err => errors.push("pageerror: " + err.message));
  page.on("requestfailed", request => {
    if (/\.(?:wasm|xml|js)(?:\?|$)/.test(request.url()))
      errors.push("requestfailed: " + request.url() + " " + request.failure()?.errorText);
  });
  let result = { name, status: "failed", errors };
  try {
    const response = await page.goto(URL, {
      waitUntil: "domcontentloaded", timeout: 45000,
    });
    assert.equal(response?.status(), 200, "Public app did not return HTTP 200");
    await page.locator(".build-stamp a").waitFor({ timeout: 30000 });
    const href = await page.locator(".build-stamp a").getAttribute("href");
    assert.equal(href, "https://github.com/Jonamarti/humanoid-physics-lab/commit/" + expected,
      "Deployed build stamp is not the expected main commit");
    const xml = await page.evaluate(async () => {
      const response = await fetch("/humanoid-physics-lab/models/humanoid_gait_v3.xml");
      return { status: response.status, body: await response.text() };
    });
    assert.equal(xml.status, 200, "Articulated v3 model asset not deployed");
    assert(xml.body.includes('name="left_toe_geom"') &&
      xml.body.includes('name="right_toe_geom"'),
    "Production v3 asset lacks real articulated toe collision geometry");

    const details = await callback(page);
    if (errors.length) throw new Error("Browser/runtime errors: " + errors.join(" | "));
    result = { name, status: "passed", commit: expected, ...details, errors };
    console.log("DEPLOYED-SMOKE PASS " + JSON.stringify(result));
  } catch (err) {
    const status = await page.locator("#physics-status").innerText().catch(() => "");
    result = { name, status: "failed", error: String(err), physicsStatus: status, errors };
    console.error("DEPLOYED-SMOKE FAIL " + JSON.stringify(result));
  } finally {
    try {
      await page.screenshot({ path: path.join(output, name + ".png"), fullPage: true });
    } catch (err) {
      console.warn("SCREENSHOT-FAILED", name, String(err));
    }
    results.push(result);
    await context.close();
  }
}

try {
  await scenario("desktop-v3-reactive-push", { width: 1320, height: 840 }, async page => {
    await loadStanding(page);
    await page.locator("#push-direction").selectOption("forward");
    await page.locator("#push-peak").fill("120");
    await page.locator("#push-duration").fill("160");
    await page.locator("#v3-balance-mode").selectOption("whole-body-step");
    await page.locator("#v3-push-run").click();
    await advanceTo(page, 1.25);
    const status = await page.locator("#physics-status").innerText();
    assert(!status.includes("No se pudo"), "V3 push trial was rejected: " + status);
    return {
      model: "v3 articulated forefoot",
      forceN: 120,
      tReadout: await page.locator("#physics-readout").innerText(),
      physicsStatus: status,
      contactReadout: await page.locator("#v3-contact-readout").innerText(),
      stepReadout: await page.locator("#step-readout").innerText(),
      claim: "UI/WASM/integration smoke only; no physical step recovery asserted",
    };
  });

  await scenario("desktop-advanced-3d-force-editor", { width: 1320, height: 840 }, async page => {
    await loadStanding(page);
    await page.locator("#multi-force-editor input[type=checkbox]").check();
    const run = page.getByRole("button", { name: "Ejecutar secuencia" });
    await page.waitForFunction(
      () => {
        const b = [...document.querySelectorAll("#multi-force-editor button")]
          .find(x => x.textContent?.trim() === "Ejecutar secuencia");
        return b && !b.disabled;
      }, undefined, { timeout: 85000 },
    );
    await page.getByRole("button", { name: "+ Añadir fuerza" }).click();
    assert.equal(await page.locator("#multi-force-editor fieldset").count(), 2,
      "Multi-force UI did not add a second force");
    await run.click();
    await advanceTo(page, 1.25);
    return {
      forcesConfigured: 2,
      physicsStatus: await page.locator("#physics-status").innerText(),
      tReadout: await page.locator("#physics-readout").innerText(),
      stepReadout: await page.locator("#step-readout").innerText(),
      claim: "Force timeline reached actual physics runtime, not a proven recovery",
    };
  });

  await scenario("mobile-standing-view", { width: 390, height: 844 }, async page => {
    await loadStanding(page);
    await page.locator("#physics-toggle").click();
    await advanceTo(page, 0.3);
    assert.equal(await page.locator("#physics-viewport canvas").count(), 1);
    return {
      viewport: "390x844",
      tReadout: await page.locator("#physics-readout").innerText(),
      status: await page.locator("#physics-status").innerText(),
    };
  });
} finally {
  await writeFile(path.join(output, "summary.json"), JSON.stringify({
    appUrl: URL,
    expectedCommit: expected,
    results,
  }, null, 2));
  await browser.close();
}

console.log("DEPLOYED-SMOKE SUMMARY " + JSON.stringify(
  { passed: results.filter(r => r.status === "passed").length,
    total: results.length, expectedCommit: expected },
));
if (results.some(r => r.status !== "passed")) process.exitCode = 1;
