import { defineConfig } from "vitest/config";

export default defineConfig({
  test: {
    environment: "node",
    // Manual benchmarks are discovered, but skip themselves unless explicitly enabled.
    include: ["tests/unit/**/*.test.ts", "tests/manual/**/*.test.ts"],
    coverage: { include: ["src/**/*.ts"] },
  },
});
