import { defineConfig } from "vite";

// All assets and future WASM modules must resolve below the project Pages path.
export default defineConfig({
  base: "/humanoid-physics-lab/",
  build: { target: "es2022" },
});
