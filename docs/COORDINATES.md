# Convenciones físicas y coordenadas

- Sistema SI: metros, kilogramos, segundos, newtons, N·m de par, N·s de impulso, radianes.
- Se prevé MuJoCo con **Z arriba** y gravedad aprox. `[0, 0, -9.81]` m/s².
- Three.js emplea **Y arriba** por defecto: el futuro adaptador realizará la transformación explícita sin mutar coordenadas de la física.
- Pelvis/root libre con seis grados de libertad; equilibrio por articulaciones/contactos, no por fuerzas de sustentación invisibles.
- Un impacto define fuerza, duración y punto de aplicación en un marco de referencia documentado: `J = ∫ F dt` y `τ = (p - p_COM) × F`.
- El timestep físico se separará del framerate de render. Nombres y orden de articulaciones se definirán en #7.

**Estado:** convenciones acordadas; el motor no está integrado aún.
