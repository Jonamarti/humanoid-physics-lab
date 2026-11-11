import { execFileSync } from "node:child_process";
import { readFileSync } from "node:fs";
import { defineConfig } from "vite";

// Resolve metadata from the exact local checkout being built. This is not
// fetched at runtime from GitHub (where main could have moved meanwhile).
const packageVersion = (JSON.parse(
  readFileSync(new URL("./package.json", import.meta.url), "utf8"),
) as { version: string }).version;

let sourceCommit = "";
try {
  sourceCommit = execFileSync("git", ["rev-parse", "HEAD"], {
    encoding: "utf8", stdio: ["ignore", "pipe", "ignore"],
  }).trim();
} catch {
  sourceCommit = process.env.GITHUB_SHA ?? "";
}
const completeSha = /^[0-9a-f]{40}$/i.test(sourceCommit) ?
  sourceCommit.toLowerCase() : null;
const shortSha = completeSha?.slice(0, 8) ?? "local";
const builtAtUtc = new Date().toISOString();
const commitLabel = completeSha ?
  `<a href="https://github.com/Jonamarti/humanoid-physics-lab/commit/${completeSha}" rel="noopener noreferrer" target="_blank" title="Ver commit ${completeSha} en GitHub">commit ${shortSha}</a>` :
  `<span title="Sin información de Git en esta compilación">commit local</span>`;

const stamp = `<aside class="build-stamp" aria-label="Versión de esta compilación: ${packageVersion}, commit ${shortSha}" title="Compilación UTC: ${builtAtUtc}">
  <span class="build-stamp-version">v${packageVersion}</span>
  <span class="build-stamp-separator" aria-hidden="true">·</span>
  ${commitLabel}
</aside>`;

// The HTML is stamped DURING the Vite build, including GitHub Pages builds.
// Because Git is read at build time, the badge identifies the deployed
// artifact and will not silently change when the repository's main advances.
export default defineConfig({
  base: "/humanoid-physics-lab/",
  build: { target: "es2022" },
  plugins: [{
    name: "hpl-build-stamp",
    transformIndexHtml(html) {
      const anchor = "<!-- HPL_BUILD_STAMP -->";
      if (!html.includes(anchor)) {
        throw new Error("Falta el marcador HPL_BUILD_STAMP en index.html");
      }
      return html.replace(anchor, stamp);
    },
  }],
});
