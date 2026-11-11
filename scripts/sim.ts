// Punto de entrada de `npm run sim`. Toda la lógica está en `src/scenario/cli.ts`.
import { runCli } from "../src/scenario/cli";

runCli(process.argv.slice(2)).then(
  (code) => {
    process.exitCode = code;
  },
  (error: unknown) => {
    console.error(error);
    process.exitCode = 3;
  },
);
