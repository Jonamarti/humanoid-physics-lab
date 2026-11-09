/** Converts MuJoCo's Z-up world into Three.js' Y-up scene.
 * Rotation conversion must be added before rendering articulated rigs.
 */
export function mujocoToThreePosition(position: readonly [number, number, number]): [number, number, number] {
  const [x, y, z] = position;
  if (![x, y, z].every(Number.isFinite)) throw new RangeError("Non-finite physics position");
  return [x, z, -y];
}
