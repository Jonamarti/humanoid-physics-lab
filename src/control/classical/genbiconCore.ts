/** Independent reduced-model adaptation of the virtual-force and hip-reaction
 * ideas in Coros, Beaudoin, van de Panne, Generalized Biped Walking Control
 * (2010), and Cartwheel 3D, Core/IKVMCController.cpp and Core/SimBiController.cpp.
 *
 * Not a full GENBICON port. In particular, Cartwheel applies its joint
 * moment to the parent (Physics/ODEWorld.cpp); MuJoCo positive hinge torque
 * acts on the child. Use the actual world hinge axis and reverse the sign
 * for a proximal chain rooted at a planted foot. All commands are internal.
 */
import { clamp, type Pair } from "./simbiconCore";
export type Vector = readonly [number,number,number];
export function virtualForce(massKg:number,com:Vector,vel:Vector,ref:Pair):Vector {
  if(!(massKg>0)||![massKg,...com,...vel,...ref].every(Number.isFinite)) throw new RangeError("Invalid virtual force");
  // Double-stance acceleration feedback, bounded as in the published
  // implementation's horizontal force task. HPL uses X forward, Y left.
  return [clamp(massKg*(10*(ref[0]-com[0])-150*vel[0]),-60,60),
    clamp(massKg*(20*(ref[1]-com[1])-9*vel[1]),-100,100),0];
}
/** Work-conjugate moment for a proximal chain with the distal link fixed:
 * delta p = -(axis × (p - anchor)) delta q. tau = J^T F.
 * Not a force applied at the COM and not a contact constraint.
 */
export function proximalMoment(axis:Vector,anchor:Vector,point:Vector,force:Vector):number {
  if(![...axis,...anchor,...point,...force].every(Number.isFinite)) throw new RangeError("Invalid world moment");
  const x=point[0]-anchor[0],y=point[1]-anchor[1],z=point[2]-anchor[2];
  return -(axis[0]*(y*force[2]-z*force[1])+axis[1]*(z*force[0]-x*force[2])+axis[2]*(x*force[1]-y*force[0]));
}
export function measuredShares(loads:Pair):Pair {
  if(!loads.every(v=>Number.isFinite(v)&&v>=0)) throw new RangeError("Invalid measured loads");
  const sum=loads[0]+loads[1];
  return sum>1?[loads[0]/sum,loads[1]/sum]:[0,0];
}
/** Allocate residual of the desired pelvis reaction to the two hips in
 * proportion to measured normal load. Preserve the requested sum whenever
 * the limits allow it. Never write the net moment directly to the root.
 */
export function loadedHipReaction(base:Pair,desiredRootNm:number,shares:Pair,limitNm:number):{left:number;right:number;residualNm:number} {
  if(![...base,...shares,desiredRootNm,limitNm].every(Number.isFinite)||limitNm<=0||
    shares.some(v=>v<0)||Math.abs(shares[0]+shares[1]-1)>1e-8) throw new RangeError("Invalid hip reaction");
  const target=clamp(-desiredRootNm,-2*limitNm,2*limitNm);
  const correction=target-base[0]-base[1];
  const left=clamp(base[0]+shares[0]*correction,Math.max(-limitNm,target-limitNm),Math.min(limitNm,target+limitNm));
  const right=target-left;
  return {left,right,residualNm:desiredRootNm+left+right};
}
/** Center of mass of the proximal part of a support chain, using the masses
 * and inertial centers actually compiled in the model. */
export function weightedCenter(points:readonly {mass:number;position:Vector}[]):Vector {
  const m=points.reduce((s,p)=>s+p.mass,0);
  if(!(m>0)||points.some(p=>!(p.mass>0)||![p.mass,...p.position].every(Number.isFinite))) throw new RangeError("Invalid proximal mass");
  const c=[0,0,0];
  for(const p of points) for(let j=0;j<3;j++) c[j]!+=p.mass*p.position[j]!/m;
  return [c[0]!,c[1]!,c[2]!];
}
