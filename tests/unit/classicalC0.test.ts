import { gzipSync } from "node:zlib";
import { createHash } from "node:crypto";
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
      // Compact 25 Hz pose trace for offline comparative visualization.
      // Derived only from the actual MuJoCo recording, never resimulated.
      // Little-endian int16, qpos in millimetres/milliradians (quaternion
      // components scaled by 1000). Quantization error <= 0.0005 units.
      const frames=result.recording.samples.filter((_,i)=>i%4===0);
      const n=frames[0]!.qpos.length;
      const bytes=Buffer.alloc(frames.length*n*2);
      frames.forEach((frame,i)=>frame.qpos.forEach((value,j)=>{
        bytes.writeInt16LE(Math.round(value*1000),2*(i*n+j));
      }));
      console.log("CLASSICAL_VIDEO "+JSON.stringify({mode:result.metrics.mode,hz:25,frames:frames.length,n,
        sha256:createHash("sha256").update(bytes).digest("hex"),data:gzipSync(bytes).toString("base64")}));
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

describe("C2 unilateral support diagnostics",()=>{
  it("measures both weight transfers and attempts both lifts with bounded internal torques",async()=>{
    for(const mode of ["shift-left","shift-right","lift-left","lift-right","shift2-left","shift2-right","lift2-left","lift2-right"] as const) {
      const result=await runClassical(mode,mode.startsWith("shift")?9:14);
      const m=result.metrics;
      console.log("CLASSICAL_C2 "+JSON.stringify({...m,unilateral:m.unilateral?{...m.unilateral,trace:undefined}:undefined}));
      console.log("CLASSICAL_C2_TRACE "+JSON.stringify({mode,trace:m.unilateral?.trace}));
      expect(m.maxRootTorque).toBe(0);
      expect(m.maxTorqueRatio).toBeLessThanOrEqual(1+1e-12);
      expect(m.unilateral).toBeDefined();
      expect(result.recording.samples.every(s=>s.qpos.every(Number.isFinite))).toBe(true);
      if(m.unilateral?.reachedHold) expect(m.unilateral.longestSupportS).toBeGreaterThanOrEqual(2);
      if(mode.startsWith("lift")&&m.unilateral?.completed) {
        expect(m.unilateral.reachedShift).toBe(true);
        expect(m.unilateral.reachedHold).toBe(true);
      }
      // Diagnostic evidence only. A timeout/fall is a failed physical objective,
      // not a failed numerical regression. Never present it as a valid lift.
      const frames=result.recording.samples.filter((_,i)=>i%4===0);
      const n=frames[0]!.qpos.length;
      const bytes=Buffer.alloc(frames.length*n*2);
      frames.forEach((frame,i)=>frame.qpos.forEach((value,j)=>{
        bytes.writeInt16LE(Math.round(value*1000),2*(i*n+j));
      }));
      console.log("CLASSICAL_C2_VIDEO "+JSON.stringify({mode,hz:25,frames:frames.length,n,
        sha256:createHash("sha256").update(bytes).digest("hex"),data:gzipSync(bytes).toString("base64")}));
    }
  },120000);
});
