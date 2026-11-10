import type { XY, XYZ } from "../control/balanceSensors";
import { estimateContactPatchSlip, type MeasuredFootPose } from "./contactPatchSlip";

/** Load-confirmed contact tracking at the MuJoCo physics cadence.
 * The instantaneous contact *material point* may change during heel-toe
 * rolling; a world-fixed sole marker would incorrectly classify it as slip.
 *
 * All feedback is observational. This class never modifies the physics
 * state, friction, contacts, collisions or actuator targets.
 */
export interface LoadedMaterialContact {
  readonly materialSlipXYMps: XY;
  readonly tangentialSpeedMps: number;
  /** Samples since a continuous load-confirmed contact was first observed. */
  readonly consecutiveLoadedSamples: number;
  /** Smooth engagement avoids an abrupt full-strength torque at touchdown. */
  readonly feedbackRamp01: number;
}

export class MaterialContactTracker {
  private previousPose: MeasuredFootPose | undefined;
  private previousTimeS: number | undefined;
  private loadedSamples = 0;
  private filteredVelocityXYMps: XY = [0,0];

  /** At least three consecutive physics samples after initial loading (~6ms
   * for 2ms MuJoCo steps) before contact feedback becomes active.
   */
  constructor(
    readonly engageLoadN = 60,
    readonly releaseLoadN = 35,
    readonly minimumSamples = 3,
    readonly filterAlpha = 0.35,
    readonly maximumSampleGapS = 0.025,
  ) {
    if (![engageLoadN,releaseLoadN,minimumSamples,filterAlpha,
      maximumSampleGapS].every(Number.isFinite) ||
      engageLoadN<=releaseLoadN || releaseLoadN<=0 ||
      !Number.isSafeInteger(minimumSamples) || minimumSamples<2 ||
      filterAlpha<=0 || filterAlpha>1 ||
      maximumSampleGapS<=0 || maximumSampleGapS>0.25) {
      throw new RangeError("Parámetros de contacto físico inválidos");
    }
  }

  reset(): void {
    this.previousPose = undefined;
    this.previousTimeS = undefined;
    this.loadedSamples = 0;
    this.filteredVelocityXYMps = [0,0];
  }

  observe(input: {
    readonly timeS: number;
    readonly footPose: MeasuredFootPose;
    readonly contactWorldM: XYZ | null;
    readonly verticalLoadN: number;
  }): LoadedMaterialContact | null {
    const {timeS,footPose,contactWorldM,verticalLoadN}=input;
    if (!Number.isFinite(timeS) || timeS<0 || !Number.isFinite(verticalLoadN) ||
      verticalLoadN<0 || footPose.rotationWorld.length!==9 ||
      ![...footPose.centerWorldM,...footPose.rotationWorld,
        ...(contactWorldM??[])].every(Number.isFinite)) {
      throw new RangeError("Tiempo, pose o carga de contacto no válido");
    }
    if (this.previousTimeS!==undefined && timeS<=this.previousTimeS) {
      throw new RangeError("Contacto sin avance de tiempo físico");
    }
    // Contact must be visibly loaded before the first frame is accepted.
    // Once active, the lower threshold avoids chatter at ~40-60 N.
    const threshold=this.loadedSamples>0 ? this.releaseLoadN : this.engageLoadN;
    if (!contactWorldM || verticalLoadN<threshold) {
      this.reset();
      return null;
    }
    const lastPose=this.previousPose;
    const dt=this.previousTimeS===undefined ? null : timeS-this.previousTimeS;
    this.previousPose=footPose;
    this.previousTimeS=timeS;
    if (!lastPose || dt===null || dt>this.maximumSampleGapS) {
      this.loadedSamples=1;
      this.filteredVelocityXYMps=[0,0];
      return null; // never interpret first impact as stable static contact
    }
    const motion=estimateContactPatchSlip(lastPose,footPose,contactWorldM,dt);
    const filtered:XY=this.loadedSamples===1 ? motion.tangentialVelocityWorldMps :
      [
        this.filteredVelocityXYMps[0]*(1-this.filterAlpha)+
          motion.tangentialVelocityWorldMps[0]*this.filterAlpha,
        this.filteredVelocityXYMps[1]*(1-this.filterAlpha)+
          motion.tangentialVelocityWorldMps[1]*this.filterAlpha,
      ];
    this.filteredVelocityXYMps=filtered;
    this.loadedSamples++;
    if(this.loadedSamples<this.minimumSamples) return null;
    // 10 control ticks for a gradual ~20ms ramp at a 2ms timestep.
    const ramp=Math.min(1,(this.loadedSamples-this.minimumSamples+1)/10);
    return {
      materialSlipXYMps:filtered,
      tangentialSpeedMps:Math.hypot(...filtered),
      consecutiveLoadedSamples:this.loadedSamples,
      feedbackRamp01:ramp,
    };
  }
}
