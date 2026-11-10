import type { XYZ } from "../control/balanceSensors";

export interface MeasuredFootPose {
  readonly centerWorldM: XYZ;
  /** MuJoCo world-frame rotation, 3x3 ROW-MAJOR. */
  readonly rotationWorld: readonly number[];
}

export interface ContactPatchVelocity {
  /** XY velocity of the currently contacting *material point* of the foot
   * relative to a stationary floor, not the velocity of the sole center.
   */
  readonly tangentialVelocityWorldMps: readonly [number,number];
  readonly tangentialSpeedMps: number;
  /** For integration of absolute sliding distance at fixed sampling time. */
  readonly estimatedSlipDistanceThisSampleM: number;
}

/** Kinematic estimate from two successive measured MuJoCo poses and the
 * CURRENT solver-derived force-weighted contact point. Maps that material
 * point into the prior foot pose to estimate its XY displacement.
 *
 * This reduces false 'slip' when a foot rotates around its point of
 * contact: the tracked sole site can move even with static friction.
 *
 * Requires a real measured foot-floor contact and a stationary world floor.
 * Not a certified contact impulse or exact continuous sliding integral.
 * Short sample periods and true 3D centroid position improve accuracy.
 */
export function estimateContactPatchSlip(
  previous: MeasuredFootPose,
  current: MeasuredFootPose,
  contactWorldM: XYZ,
  deltaTimeS: number,
): ContactPatchVelocity {
  const vals=[...previous.centerWorldM,...current.centerWorldM,
    ...previous.rotationWorld,...current.rotationWorld,...contactWorldM,deltaTimeS];
  if (previous.rotationWorld.length !== 9 || current.rotationWorld.length !== 9 ||
      !vals.every(Number.isFinite) || deltaTimeS <= 0 || deltaTimeS > 0.25) {
    throw new RangeError("Poses, contacto o cadencia inválidos");
  }
  const r=current.rotationWorld, p=previous.rotationWorld;
  const dx=contactWorldM[0]-current.centerWorldM[0];
  const dy=contactWorldM[1]-current.centerWorldM[1];
  const dz=contactWorldM[2]-current.centerWorldM[2];
  const localX=r[0]!*dx+r[3]!*dy+r[6]!*dz;
  const localY=r[1]!*dx+r[4]!*dy+r[7]!*dz;
  const localZ=r[2]!*dx+r[5]!*dy+r[8]!*dz;
  const prevX=previous.centerWorldM[0]+p[0]!*localX+p[1]!*localY+p[2]!*localZ;
  const prevY=previous.centerWorldM[1]+p[3]!*localX+p[4]!*localY+p[5]!*localZ;
  const vx=(contactWorldM[0]-prevX)/deltaTimeS;
  const vy=(contactWorldM[1]-prevY)/deltaTimeS;
  const speed=Math.hypot(vx,vy);
  return {
    tangentialVelocityWorldMps:[vx,vy],
    tangentialSpeedMps:speed,
    estimatedSlipDistanceThisSampleM:speed*deltaTimeS,
  };
}
