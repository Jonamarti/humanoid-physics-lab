/** A free rigid sphere falling onto a fixed plane.
 * This is a real contact/gravity integration smoke scene, NOT a humanoid.
 */
export const GRAVITY_DEMO_MJCF = `
<mujoco model="gravity_demo">
  <compiler angle="radian"/>
  <option timestep="0.002" gravity="0 0 -9.81" integrator="Euler"/>
  <worldbody>
    <geom name="floor" type="plane" size="4 4 0.1" friction="0.8 0.02 0.001"/>
    <body name="falling_sphere" pos="0 0 1.5">
      <freejoint name="free"/>
      <geom name="sphere" type="sphere" size="0.12" mass="1" rgba="0.3 0.8 1 1"/>
    </body>
  </worldbody>
</mujoco>`;
