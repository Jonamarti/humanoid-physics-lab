import {
  candidateFootCornersXY, convexHullXY, type FootGeom, type XY, type XYZ,
} from "./balanceSensors";

/** Only the solved floor-vs-foot normal force counts as support. This
 * intentionally excludes non-contacting near-ground geoms and body contacts.
 */
export interface GroundContactSample {
  readonly geom1: number;
  readonly geom2: number;
  readonly positionWorldM: XYZ;
  /** Positive scalar from mj_contactForce, in the contact-normal direction. */
  readonly contactNormalForceN: number;
  /** First axis of mjContact.frame, pointing from geom1 to geom2. */
  readonly contactNormalWorld: XYZ;
  /** Solver-derived tangential force, in world space, before geom ordering. */
  readonly tangentForceWorldN?: XYZ;
  /** Fraction of the solver contact friction cone/pyramid used. */
  readonly frictionUtilization?: number | null;
}

export interface FootSupport {
  readonly centerXY: XY;
  /** Full projected foot rectangle for predicting a *future* landing. */
  readonly footprintXY: readonly XY[];
  readonly verticalLoadN: number;
  /** Net horizontal frictional reaction on this foot, from solved tangents. */
  readonly tangentialReactionXYN: XY;
  /** Normal-force-weighted mean friction usage (0 to ~1 in cone). */
  readonly frictionUtilizationMean: number | null;
  readonly frictionUtilizationPeak: number | null;
  readonly activeContacts: number;
  /** Vertical-force-weighted contact position; not a full 6D CoP. */
  readonly verticalLoadCentroidXY: XY | null;
  /** Actual 3D force-weighted contact centroid from MuJoCo, not a CoP. */
  readonly verticalLoadCentroidWorldM: XYZ | null;
  readonly isLoaded: boolean;
}

export interface GroundSupportObservation {
  readonly left: FootSupport;
  readonly right: FootSupport;
  /** Hull of feet that carry solved upward force, not merely near the ground. */
  readonly loadedSupportPolygonXY: readonly XY[];
  readonly totalVerticalLoadN: number;
  readonly activeFootContacts: number;
}

export function summarizeGroundSupport(input: {
  readonly floorGeomId: number;
  readonly leftFootGeomId: number;
  readonly rightFootGeomId: number;
  readonly leftFoot: FootGeom;
  readonly rightFoot: FootGeom;
  readonly contacts: readonly GroundContactSample[];
  readonly minimumLoadedForceN?: number;
}): GroundSupportObservation {
  const {
    floorGeomId, leftFootGeomId, rightFootGeomId,
    leftFoot, rightFoot, contacts,
  } = input;
  const threshold = input.minimumLoadedForceN ?? 10;
  if (![floorGeomId, leftFootGeomId, rightFootGeomId].every(Number.isSafeInteger) ||
      new Set([floorGeomId, leftFootGeomId, rightFootGeomId]).size !== 3 ||
      !Number.isFinite(threshold) || threshold <= 0) {
    throw new RangeError("Geometrías o umbral de contacto inválidos");
  }
  type Bucket = { load: number; count: number; x: number; y: number; z: number;
    tx: number; ty: number; frictionWeighted: number; frictionLoad: number;
    frictionPeak: number | null };
  const makeBucket = (): Bucket => ({
    load: 0,count: 0,x: 0,y: 0,z: 0,tx: 0,ty: 0,
    frictionWeighted: 0,frictionLoad: 0,frictionPeak: null,
  });
  const buckets: Record<"left" | "right",Bucket> = {
    left: makeBucket(),right: makeBucket(),
  };
  for (const contact of contacts) {
    const { geom1,geom2,positionWorldM,contactNormalForceN,contactNormalWorld,
      tangentForceWorldN,frictionUtilization } = contact;
    if (![geom1, geom2].every(Number.isSafeInteger) ||
        ![...positionWorldM, ...contactNormalWorld, contactNormalForceN].every(Number.isFinite) ||
        contactNormalForceN < 0 ||
        (tangentForceWorldN !== undefined &&
          !tangentForceWorldN.every(Number.isFinite)) ||
        (frictionUtilization != null &&
          (!Number.isFinite(frictionUtilization) || frictionUtilization < 0))) {
      throw new RangeError("Contacto de MuJoCo inválido");
    }
    if (geom1 !== floorGeomId && geom2 !== floorGeomId) continue;
    const footGeomId = geom1 === floorGeomId ? geom2 : geom1;
    const side = footGeomId === leftFootGeomId ? "left" :
      footGeomId === rightFootGeomId ? "right" : null;
    if (!side) continue;
    // The normal points from geom1 toward geom2. When the foot is geom1,
    // invert it to get the force on that foot. The vertical projection is
    // what bears body weight; a horizontal wall normal is not support.
    const towardFoot = geom2 === footGeomId ? 1 : -1;
    const upwardN = Math.max(0, towardFoot * contactNormalWorld[2] * contactNormalForceN);
    if (upwardN <= 0) continue;
    const b = buckets[side];
    b.load += upwardN;
    b.count++;
    b.x += positionWorldM[0] * upwardN;
    b.y += positionWorldM[1] * upwardN;
    b.z += positionWorldM[2] * upwardN;
    if (tangentForceWorldN) {
      b.tx += towardFoot*tangentForceWorldN[0];
      b.ty += towardFoot*tangentForceWorldN[1];
    }
    if (frictionUtilization != null && upwardN > 5) {
      b.frictionLoad += upwardN;
      b.frictionWeighted += frictionUtilization*upwardN;
      b.frictionPeak = Math.max(b.frictionPeak ?? 0,frictionUtilization);
    }
  }

  const makeFoot = (geom: FootGeom, side: "left" | "right"): FootSupport => {
    const b = buckets[side];
    const footprint = candidateFootCornersXY(geom, 10); // projection, NOT contact
    return {
      centerXY: [geom.centerM[0], geom.centerM[1]],
      footprintXY: footprint,
      verticalLoadN: b.load,
      tangentialReactionXYN: [b.tx,b.ty],
      frictionUtilizationMean: b.frictionLoad > 0 ?
        b.frictionWeighted/b.frictionLoad : null,
      frictionUtilizationPeak: b.frictionPeak,
      activeContacts: b.count,
      verticalLoadCentroidXY: b.load > 0 ? [b.x / b.load, b.y / b.load] : null,
      verticalLoadCentroidWorldM: b.load > 0 ?
        [b.x / b.load, b.y / b.load, b.z / b.load] : null,
      isLoaded: b.load >= threshold,
    };
  };
  const left = makeFoot(leftFoot, "left");
  const right = makeFoot(rightFoot, "right");
  return {
    left, right,
    loadedSupportPolygonXY: convexHullXY([
      ...(left.isLoaded ? left.footprintXY : []),
      ...(right.isLoaded ? right.footprintXY : []),
    ]),
    totalVerticalLoadN: left.verticalLoadN + right.verticalLoadN,
    activeFootContacts: left.activeContacts + right.activeContacts,
  };
}
