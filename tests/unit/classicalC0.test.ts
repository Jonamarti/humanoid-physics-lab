import { describe, expect, it } from "vitest";
import { advanceGait, initialGait, hipPair, pd, swingPitchTarget } from "../../src/control/classical/simbiconCore";
import { runClassical } from "../../src/control/classical/experiment";
import { serializeRecording } from "../../src/recording/write";
import { parseRecording } from "../../src/recording/read";
import { RecordingPlayer } from "../../src/recording/player";

describe("Classical C0 primitives", () => {
  it("allocates internal hip torques and exposes saturation", () => {
    expect(hipPair(50, 20, 100)).toEqual({stanceNm:-70,swingNm:20,residualNm:0});
    expect(hipPair(250, 20, 100)).toEqual({stanceNm:-100,swingNm:-100,residualNm:50});
    expect(pd(1,0,0,500,20,100)).toBe(100);
    expect(swingPitchTarget(-.2,.1,.2)).toBeLessThan(-.2);
  });
  it("does not mistake continuous floor contact for a step", () => {
    let state=initialGait();
    for(let i=0;i<130;i++) state=advanceGait(state,{loadN:[300,300],clearanceM:[0,0]},.01);
    expect(state.phase).toBe("timeout");
    expect(state.steps).toBe(0);
  });
  it("requires unload, clearance, and sustained landing before alternating", () => {
    let state=initialGait();
    for(let i=0;i<40;i++) state=advanceGait(state,{loadN:[0,700],clearanceM:[.04,0]},.01);
    expect(state.steps).toBe(0);
    for(let i=0;i<7;i++) state=advanceGait(state,{loadN:[300,400],clearanceM:[0,0]},.01);
    expect(state.steps).toBe(1);
    expect(state.swing).toBe("right");
  });
});
describe("Classical C0 MuJoCo experiment (diagnostic, not gait validation)",()=>{
  it("records identical repeat runs, bounds torque and exposes actual fall/step metrics",async()=>{
    const first=await runClassical("stand",3);
    const second=await runClassical("stand",3);
    expect(first.metrics).toEqual(second.metrics);
    expect(serializeRecording(first.recording)).toBe(serializeRecording(second.recording));
    const passive=await runClassical("passive",3);
    const walk=await runClassical("walk",3);
    const balance=await runClassical("balance",3);
    for(const result of [first,passive,walk,balance]){
      console.log("CLASSICAL_C0 "+JSON.stringify(result.metrics));
      expect(result.metrics.maxRootTorque).toBe(0);
      expect(result.metrics.maxTorqueRatio).toBeLessThanOrEqual(1+1e-12);
      expect(result.recording.samples).toHaveLength(301);
      expect(result.recording.samples.every(s=>s.qpos.every(Number.isFinite))).toBe(true);
      const decoded=await parseRecording(serializeRecording(result.recording));
      const player=new RecordingPlayer(decoded);
      expect(player.sampleAt(0).qpos).toEqual(decoded.samples[0]!.qpos);
      expect(player.sampleAt(3).qpos).toEqual(decoded.samples[300]!.qpos);
    }
    expect(passive.metrics.maxTorqueRatio).toBe(0);
    expect(passive.metrics.steps).toBe(0);
    expect(passive.metrics.firstFallS).not.toBeNull();
  },30000);
});

describe("C1 nominal standing endurance",()=>{
  it("holds the free-root reduced body for sixty seconds without assistance",async()=>{
    const result=await runClassical("balance",60);
    const m=result.metrics;
    console.log("CLASSICAL_C1 "+JSON.stringify(m));
    expect(m.firstFallS).toBeNull();
    expect(m.maxRootTorque).toBe(0);
    expect(m.maxTorqueRatio).toBeLessThanOrEqual(1+1e-12);
    expect(m.maxTiltDeg).toBeLessThan(5);
    expect(m.minPelvisM).toBeGreaterThan(.85);
    expect(m.finalComSpeedMps).toBeLessThan(.1);
    expect(m.maxGroundPenetrationM).toBeLessThan(.005);
    expect(m.maxLoadedMarkerDriftM).toBeLessThan(.02);
    expect(result.recording.samples).toHaveLength(6001);
    // Marker drift is a screening metric, not material-contact slip.
    // Perturbation robustness and visual naturalness remain unvalidated.
  },120000);
});
