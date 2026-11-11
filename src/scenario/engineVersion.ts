import { readFileSync } from "node:fs";
import { join } from "node:path";

/** Versión de `package.json` del directorio de trabajo (raíz del repositorio con `npm`/`vitest`); `unknown` si no se lee. */
export function readEngineVersion(): string {
  try {
    const pkg = JSON.parse(readFileSync(join(process.cwd(), "package.json"), "utf8")) as { version?: unknown };
    return typeof pkg.version === "string" ? pkg.version : "unknown";
  } catch {
    return "unknown";
  }
}
