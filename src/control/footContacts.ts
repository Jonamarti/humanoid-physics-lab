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
  /** Optional solved segment breakdown in the articulated-foot model. */
  readonly segments?: Readonly<Record<"rearfoot" | "forefoot", FootSegmentSupport>>;
}

/** Segment-specific load and centroid: do not confuse contact migration with slip. */
export interface FootSegmentSupport {
  readonly verticalLoadN: number;
  readonly tangentialReactionXYN: XY;
  readonly verticalLoadCentroidWorldM: XYZ | null;
  readonly activeContacts: number;
  readonly frictionUtilizationMean: number | null;
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
  /** Both toe geom IDs and poses must be provided together for v3. */
  readonly leftToeGeomId?: number;
  readonly rightToeGeomId?: number;
  readonly leftToe?: FootGeom;
  readonly rightToe?: FootGeom;
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
  const segmented = [input.leftToeGeomId,input.rightToeGeomId,input.leftToe,input.rightToe]
    .some(v => v !== undefined);
  if (segmented && [input.leftToeGeomId,input.rightToeGeomId,input.leftToe,input.rightToe]
      .some(v => v === undefined)) {
    throw new RangeError("Segmentos de antepié incompletos");
  }
  const geomIds = [floorGeomId,leftFootGeomId,rightFootGeomId,
    ...(segmented ? [input.leftToeGeomId!,input.rightToeGeomId!] : [])];
  if (!geomIds.every(Number.isSafeInteger) ||
      new Set(geomIds).size !== geomIds.length ||
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
  const segments = {
    left: { rearfoot:makeBucket(),forefoot:makeBucket() },
    right: { rearfoot:makeBucket(),forefoot:makeBucket() },
  };
  const add = (b:Bucket,load:number,contact:GroundContactSample,towardFoot:number):void => {
    b.load += load;
    b.count++;
    b.x += contact.positionWorldM[0]*load;
    b.y += contact.positionWorldM[1]*load;
    b.z += contact.positionWorldM[2]*load;
    if (contact.tangentForceWorldN) {
      b.tx += towardFoot*contact.tangentForceWorldN[0];
      b.ty += towardFoot*contact.tangentForceWorldN[1];
    }
    if (contact.frictionUtilization != null && load > 5) {
      b.frictionLoad += load;
      b.frictionWeighted += contact.frictionUtilization*load;
      b.frictionPeak = Math.max(b.frictionPeak ?? 0,contact.frictionUtilization);
    }
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
    const side = footGeomId === leftFootGeomId ||
      (segmented && footGeomId === input.leftToeGeomId) ? "left" :
      footGeomId === rightFootGeomId ||
      (segmented && footGeomId === input.rightToeGeomId) ? "right" : null;
    if (!side) continue;
    const part = footGeomId === leftFootGeomId || footGeomId === rightFootGeomId ?
      "rearfoot" : "forefoot";
    // The normal points from geom1 toward geom2. When the foot is geom1,
    // invert it to get the force on that foot. The vertical projection is
    // what bears body weight; a horizontal wall normal is not support.
    const towardFoot = geom2 === footGeomId ? 1 : -1;
    const upwardN = Math.max(0, towardFoot * contactNormalWorld[2] * contactNormalForceN);
    if (upwardN <= 0) continue;
    add(buckets[side],upwardN,contact,towardFoot);
    add(segments[side][part],upwardN,contact,towardFoot);
  }

  const makeFoot = (geom: FootGeom, side: "left" | "right"): FootSupport => {
    const b = buckets[side];
    const toe = side==="left" ? input.leftToe : input.rightToe;
    const footprint = convexHullXY([
      ...candidateFootCornersXY(geom, 10),
      ...(toe ? candidateFootCornersXY(toe,10) : []),
    ]); // projected candidate hull, NOT evidence of load
    const segment = (part:"rearfoot"|"forefoot"):FootSegmentSupport => {
      const c = segments[side][part];
      return {
        verticalLoadN:c.load,
        tangentialReactionXYN:[c.tx,c.ty],
        verticalLoadCentroidWorldM:c.load>0 ?
          [c.x/c.load,c.y/c.load,c.z/c.load] : null,
        activeContacts:c.count,
        frictionUtilizationMean:c.frictionLoad>0 ? c.frictionWeighted/c.frictionLoad : null,
        isLoaded:c.load>=threshold,
      };
    };
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
      ...(segmented ? {segments:{rearfoot:segment("rearfoot"),forefoot:segment("forefoot")}} : {}),
    };
  };
  const left = makeFoot(leftFoot, "left");
  const right = makeFoot(rightFoot, "right");
  return {
    left, right,
    loadedSupportPolygonXY: convexHullXY(segmented ? [
      ...(left.segments!.rearfoot.isLoaded ? candidateFootCornersXY(leftFoot,10) : []),
      ...(left.segments!.forefoot.isLoaded ? candidateFootCornersXY(input.leftToe!,10) : []),
      ...(right.segments!.rearfoot.isLoaded ? candidateFootCornersXY(rightFoot,10) : []),
      ...(right.segments!.forefoot.isLoaded ? candidateFootCornersXY(input.rightToe!,10) : []),
    ] : [
      ...(left.isLoaded ? left.footprintXY : []),
      ...(right.isLoaded ? right.footprintXY : []),
    ]),
    totalVerticalLoadN: left.verticalLoadN + right.verticalLoadN,
    activeFootContacts: left.activeContacts + right.activeContacts,
  };
}
