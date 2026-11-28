/** Geometric Jacobians and mass matrix for a free-root, hinge-only tree.
 * Independent of MuJoCo's sparse-matrix WASM output marshalling.
 * Free-joint linear velocity is world-frame; angular velocity is root-local.
 * Each body inertia is principal-axis diagonal in its inertial frame.
 */
export type V3=readonly [number,number,number];
export interface ArticulatedBody {parent:number;mass:number;inertia:V3;center:V3;inertialRotation:readonly number[]}
export interface ArticulatedHinge {body:number;dof:number;axis:V3;anchor:V3}
export interface ArticulatedGeometry {
  nv:number;rootBody:number;rootPosition:V3;rootRotation:readonly number[];
  bodies:readonly ArticulatedBody[];hinges:readonly ArticulatedHinge[];armature:readonly number[];
}
export interface Jacobian {linear:number[][];angular:number[][]}
const cross=(a:V3,b:V3):V3=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const sub=(a:V3,b:V3):V3=>[a[0]-b[0],a[1]-b[1],a[2]-b[2]];
const col=(a:readonly number[],j:number):V3=>[a[j]!,a[j+3]!,a[j+6]!];
export function pointJacobian(g:ArticulatedGeometry,body:number,point:V3):Jacobian {
  const n=g.nv;
  if(n<6||!Number.isInteger(n)||g.rootRotation.length!==9||!Number.isInteger(body)||body<1||body>=g.bodies.length)throw new RangeError("Invalid tree Jacobian");
  const linear=Array.from({length:3},()=>Array<number>(n).fill(0));
  const angular=Array.from({length:3},()=>Array<number>(n).fill(0));
  function column(d:number,v:V3,w:V3){
    for(let i=0;i<3;i++){linear[i]![d]=v[i]!;angular[i]![d]=w[i]!;}
  }
  const ancestors=new Set<number>();
  for(let b=body;b>0;b=g.bodies[b]!.parent){
    if(ancestors.has(b))throw new Error("Cycle in articulated tree");
    ancestors.add(b);
  }
  if(!ancestors.has(g.rootBody))throw new Error("Body not in free-root tree");
  for(let i=0;i<3;i++){
    const axis=col(g.rootRotation,i);
    linear[i]![i]=1;
    column(3+i,cross(axis,sub(point,g.rootPosition)),axis);
  }
  for(const j of g.hinges)if(ancestors.has(j.body)){
    if(j.dof<6||j.dof>=n)throw new RangeError("Invalid hinge dof");
    column(j.dof,cross(j.axis,sub(point,j.anchor)),j.axis);
  }
  return {linear,angular};
}
export function articulatedMass(g:ArticulatedGeometry):{mass:number[][];gravity:number[];comJacobian:number[][];com:V3;totalMass:number} {
  const n=g.nv;
  if(g.armature.length!==n||g.armature.some(x=>x<0||!Number.isFinite(x)))throw new RangeError("Invalid armature");
  const mass=Array.from({length:n},(_,i)=>Array.from({length:n},(_,j)=>i===j?g.armature[i]!:0));
  const gravity=Array<number>(n).fill(0),comJacobian=Array.from({length:3},()=>Array<number>(n).fill(0)),com=[0,0,0];
  let totalMass=0;
  for(let b=1;b<g.bodies.length;b++){
    const body=g.bodies[b]!;
    if(!(body.mass>0)||![body.mass,...body.inertia,...body.center,...body.inertialRotation].every(Number.isFinite)||
      body.inertia.some(v=>v<=0)||body.inertialRotation.length!==9)throw new RangeError("Invalid body inertia");
    const J=pointJacobian(g,b,body.center);
    totalMass+=body.mass;
    for(let k=0;k<3;k++)com[k]!+=body.mass*body.center[k]!;
    // Project the world angular Jacobian onto each principal inertia axis.
    const local=Array.from({length:3},(_,k)=>{
      const axis=col(body.inertialRotation,k);
      return Array.from({length:n},(_,d)=>axis.reduce((s,v,i)=>s+v*J.angular[i]![d]!,0));
    });
    for(let i=0;i<n;i++){
      gravity[i]!+=body.mass*9.81*J.linear[2]![i]!;
      for(let k=0;k<3;k++)comJacobian[k]![i]!+=body.mass*J.linear[k]![i]!;
      for(let j=0;j<=i;j++){
        let v=0;
        for(let k=0;k<3;k++)v+=body.mass*J.linear[k]![i]!*J.linear[k]![j]!+body.inertia[k]!*local[k]![i]!*local[k]![j]!;
        mass[i]![j]!+=v;if(i!==j)mass[j]![i]!+=v;
      }
    }
  }
  if(!(totalMass>0))throw new Error("Empty articulated tree");
  return {mass,gravity,comJacobian:comJacobian.map(row=>row.map(v=>v/totalMass)),com:[com[0]!/totalMass,com[1]!/totalMass,com[2]!/totalMass],totalMass};
}
