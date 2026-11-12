// Punto de entrada de `npm run gallery`. Toda la lógica está en `src/recording/galleryBuild.ts`.
import { buildGallery } from "../src/recording/galleryBuild";

buildGallery({ log: (line) => process.stdout.write(`${line}\n`) }).then(
  () => {
    process.exitCode = 0;
  },
  (error: unknown) => {
    console.error(error instanceof Error ? error.message : error);
    process.exitCode = 1;
  },
);
