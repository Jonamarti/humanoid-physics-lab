/** All values use SI units (N, s and N*s). No physics solver is implemented here. */
export function impulseFromConstantForce(forceNewtons: number, durationSeconds: number): number {
  if (!Number.isFinite(forceNewtons) || !Number.isFinite(durationSeconds)) {
    throw new RangeError("Force and duration must be finite");
  }
  if (durationSeconds < 0) {
    throw new RangeError("Duration cannot be negative");
  }
  return forceNewtons * durationSeconds;
}

export function clamp(value: number, lower: number, upper: number): number {
  if (![value, lower, upper].every(Number.isFinite) || lower > upper) {
    throw new RangeError("Invalid clamp bounds or value");
  }
  return Math.min(Math.max(value, lower), upper);
}
