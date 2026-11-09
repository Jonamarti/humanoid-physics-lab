/** Fixed step scheduler; render time never changes the physics timestep. */
export class FixedClock {
  private accumulator = 0;
  constructor(readonly timestep = 0.002, readonly maxCatchupSteps = 50) {
    if (!Number.isFinite(timestep) || timestep <= 0 ||
        !Number.isSafeInteger(maxCatchupSteps) || maxCatchupSteps <= 0) {
      throw new RangeError("Invalid fixed timestep configuration");
    }
  }
  reset(): void {
    this.accumulator = 0;
  }
  consume(elapsedSeconds: number): number {
    if (!Number.isFinite(elapsedSeconds) || elapsedSeconds < 0) {
      throw new RangeError("Invalid elapsed time");
    }
    // Safety cap: never try to simulate hours after a tab resumes.
    this.accumulator = Math.min(this.accumulator + elapsedSeconds,
      this.timestep * this.maxCatchupSteps);
    const steps = Math.floor((this.accumulator + 1e-12) / this.timestep);
    this.accumulator = Math.max(0, this.accumulator - steps * this.timestep);
    return steps;
  }
}
