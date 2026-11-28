/** Deterministic dense constrained least-squares kernel for offline WBC.
 * min 1/2 ||A*x-b||^2 + 1/2*reg*||x||^2, l <= C*x <= u.
 * Deterministic active-set KKT solve first, scaled ADMM fallback; no training,
 * random sampling, external dependencies or root actuation. The caller
 * must reject infeasible or unconverged results, never silently clip them.
 */
export interface LinearRow {a:readonly number[];b:number;weight?:number}
export interface BoundRow {a:readonly number[];lo:number;hi:number}
export interface BoxLeastSquaresProblem {n:number;tasks:readonly LinearRow[];constraints:readonly BoundRow[];regularization?:number;rho?:number;iterations?:number;tolerance?:number}
export interface BoxSolution {x:number[];iterations:number;converged:boolean;maxViolation:number;primalResidual:number;dualResidual:number;method:"active-set"|"admm"}
const dot=(a:readonly number[],b:readonly number[]):number=>a.reduce((s,v,i)=>s+v*b[i]!,0);
function cholesky(a:number[][]):number[][] {
  const n=a.length,L=Array.from({length:n},()=>Array<number>(n).fill(0));
  for(let i=0;i<n;i++)for(let j=0;j<=i;j++){
    let v=a[i]![j]!;
    for(let k=0;k<j;k++)v-=L[i]![k]!*L[j]![k]!;
    if(i===j){if(!(v>1e-14)||!Number.isFinite(v))throw new Error("Non-positive QP Hessian");L[i]![j]=Math.sqrt(v);}
    else L[i]![j]=v/L[j]![j]!;
  }
  return L;
}
function solve(L:number[][],rhs:number[]):number[]{
  const n=rhs.length,y=rhs.slice(),x=rhs.slice();
  for(let i=0;i<n;i++){for(let j=0;j<i;j++)y[i]!-=L[i]![j]!*y[j]!;y[i]!/=L[i]![i]!;}
  for(let i=n-1;i>=0;i--){x[i]=y[i]!;for(let j=i+1;j<n;j++)x[i]!-=L[j]![i]!*x[j]!;x[i]!/=L[i]![i]!;}
  return x;
}
/** Fast path for strictly convex problems with few active constraints.
 * Solve the Schur complement of the equality-constrained optimum, add
 * the most violated bound, or drop an inequality with the wrong multiplier.
 * Dependent rows/cycling/poor conditioning fall back to ADMM. Acceptance
 * checks the actual primal and dual KKT residuals at the original tolerance.
 */
function activeSet(H:number[][],g:number[],constraints:readonly BoundRow[],tol:number):BoxSolution|null {
  const active:Array<{index:number;side:"eq"|"lo"|"hi";b:number}>=[];
  const maxSteps=Math.min(48,2*constraints.length+4);
  for(let i=0;i<constraints.length;i++){
    const c=constraints[i]!;
    if(c.lo===c.hi)active.push({index:i,side:"eq",b:c.lo});
  }
  try{
    const L=cholesky(H),free=solve(L,g);
    for(let iteration=1;iteration<=maxSteps;iteration++){
      const rows=active.map(c=>constraints[c.index]!.a);
      const columns=rows.map(a=>solve(L,Array.from(a)));
      const schur=rows.map((row,i)=>rows.map((_,j)=>.5*(dot(row,columns[j]!)+dot(rows[j]!,columns[i]!))));
      const lambda=rows.length?solve(cholesky(schur),rows.map((a,i)=>dot(a,free)-active[i]!.b)):[];
      const x=free.map((v,j)=>v-columns.reduce((s,c,k)=>s+c[j]!*lambda[k]!,0));
      if(![...x,...lambda].every(Number.isFinite))return null;
      let worst=-1,wrong=tol;
      for(let k=0;k<active.length;k++){
        const side=active[k]!.side;
        const v=side==="lo"?lambda[k]!:side==="hi"?-lambda[k]!:0;
        if(v>wrong){wrong=v;worst=k;}
      }
      if(worst>=0){active.splice(worst,1);continue;}
      let violation=0,index=-1,side:"lo"|"hi"="lo";
      for(let k=0;k<constraints.length;k++){
        const r=constraints[k]!,v=dot(r.a,x);
        const lo=r.lo-v,hi=v-r.hi;
        if(lo>violation){violation=lo;index=k;side="lo";}
        if(hi>violation){violation=hi;index=k;side="hi";}
      }
      if(violation>tol){
        if(index<0||active.some(a=>a.index===index))return null;
        const c=constraints[index]!;
        active.push({index,side,b:side==="lo"?c.lo:c.hi});continue;
      }
      const stationarity=H.map((row,i)=>dot(row,x)-g[i]!+rows.reduce((s,a,k)=>s+a[i]!*lambda[k]!,0));
      const dual=Math.max(0,...stationarity.map(Math.abs));
      if(dual>tol)return null;
      return {x,iterations:iteration,converged:true,maxViolation:violation,primalResidual:violation,dualResidual:dual,method:"active-set"};
    }
  }catch{
    // Singular Schur complement is expected for redundant active rows.
    // The independently checked ADMM fallback handles such cases.
  }
  return null;
}
export function constrainedLeastSquares(p:BoxLeastSquaresProblem):BoxSolution {
  const n=p.n,rho=p.rho??4,reg=p.regularization??1e-6,maxIter=p.iterations??1000,tol=p.tolerance??1e-6;
  if(!Number.isInteger(n)||n<1||n>256||!Number.isInteger(maxIter)||maxIter<1||maxIter>20000||
    ![rho,reg,tol].every(v=>Number.isFinite(v)&&v>0))throw new RangeError("Invalid least-squares configuration");
  const valid=(r:readonly number[])=>r.length===n&&r.every(Number.isFinite);
  const H=Array.from({length:n},(_,i)=>Array.from({length:n},(_,j)=>i===j?reg:0)),g=Array<number>(n).fill(0);
  function outer(a:readonly number[],w:number){for(let i=0;i<n;i++)for(let j=0;j<=i;j++){const v=w*a[i]!*a[j]!;H[i]![j]!+=v;if(i!==j)H[j]![i]!+=v;}}
  for(const t of p.tasks){
    const w=t.weight??1;
    if(!valid(t.a)||!Number.isFinite(t.b)||!Number.isFinite(w)||w<0)throw new RangeError("Invalid task");
    outer(t.a,w);for(let i=0;i<n;i++)g[i]!+=w*t.a[i]!*t.b;
  }
  for(const c of p.constraints){
    if(!valid(c.a)||Number.isNaN(c.lo)||Number.isNaN(c.hi)||c.lo>c.hi||c.lo===Infinity||c.hi===-Infinity)throw new RangeError("Invalid constraint");
  }
  const fast=activeSet(H,g,p.constraints,tol);
  if(fast)return fast;
  for(const c of p.constraints)outer(c.a,rho);
  const L=cholesky(H),m=p.constraints.length;
  const clip=(v:number,c:BoundRow)=>Math.max(c.lo,Math.min(c.hi,v));
  let x=Array<number>(n).fill(0);
  const z=p.constraints.map(c=>clip(0,c)),u=Array<number>(m).fill(0);
  let primal=Infinity,dual=Infinity,iteration=0;
  for(iteration=1;iteration<=maxIter;iteration++){
    const rhs=g.slice();
    for(let k=0;k<m;k++){const a=p.constraints[k]!.a,f=rho*(z[k]!-u[k]!);for(let j=0;j<n;j++)rhs[j]!+=a[j]!*f;}
    x=solve(L,rhs);
    const old=z.slice(),dr=Array<number>(n).fill(0);
    primal=0;
    for(let k=0;k<m;k++){
      const c=p.constraints[k]!,cx=dot(c.a,x);
      z[k]=clip(cx+u[k]!,c);
      u[k]!+=cx-z[k]!;
      primal=Math.max(primal,Math.abs(cx-z[k]!));
      for(let j=0;j<n;j++)dr[j]!+=rho*c.a[j]!*(z[k]!-old[k]!);
    }
    dual=dr.reduce((s,v)=>Math.max(s,Math.abs(v)),0);
    if(!x.every(Number.isFinite)||!u.every(Number.isFinite))throw new Error("Nonfinite QP iterate");
    if(primal<=tol&&dual<=tol)break;
  }
  const violation=p.constraints.reduce((v,c)=>{const a=dot(c.a,x);return Math.max(v,c.lo-a,a-c.hi);},0);
  return {x,iterations:Math.min(iteration,maxIter),converged:primal<=tol&&dual<=tol&&violation<=tol,maxViolation:violation,primalResidual:primal,dualResidual:dual,method:"admm"};
}
export interface ContactDynamicsProblem {
  mass:readonly (readonly number[])[];bias:readonly number[];
  rootDofs:number;torqueLimits:readonly number[];
  /** A point contact's translational Jacobian (world X,Y,Z). */
  contacts:readonly {jacobian:readonly (readonly number[])[];mu:number;maxNormalN:number;accelerationBias?:readonly number[];targetAcceleration?:readonly number[]}[];
  /** Acceleration tasks in generalized coordinates, e.g. COM, torso, swing and posture. */
  tasks:readonly LinearRow[];
}
export interface ContactDynamicsSolution {
  acceleration:number[];torques:number[];forces:number[][];rootResidual:number[];
  maxRootResidual:number;maxTorqueViolation:number;maxFrictionViolation:number;
  contactAccelerationResidual:number;solver:BoxSolution;feasible:boolean;
}
/** Variables: generalized accelerations (10 rad/m/s^2 scale), then
 * point forces (100 N scale). Eliminate actuator torques using
 * M*qdd + bias - sum(J^T*f). Root equations are equality constraints;
 * torque bounds and a conservative friction pyramid are inequalities.
 * Contact acceleration is a soft task to avoid inconsistent rigid
 * equalities for compliant contacts. A solved wrench is a prediction
 * only: the simulator receives the resulting internal torques, not f.
 */
export function solveContactDynamics(p:ContactDynamicsProblem):ContactDynamicsSolution {
  const n=p.bias.length,r=p.rootDofs,m=p.contacts.length,total=n+3*m;
  if(!Number.isInteger(r)||r<1||r>n||p.mass.length!==n||p.mass.some(a=>a.length!==n||!a.every(Number.isFinite))||
    !p.bias.every(Number.isFinite)||p.torqueLimits.length!==n-r||p.torqueLimits.some(t=>!(t>0)||!Number.isFinite(t)))throw new RangeError("Invalid articulated dynamics");
  const scaleQ=10,scaleF=100;
  const rows:BoundRow[]=[],tasks:LinearRow[]=[];
  const zeros=()=>Array<number>(total).fill(0);
  const dynamics=Array.from({length:n},(_,i)=>{
    const a=zeros();for(let j=0;j<n;j++)a[j]=p.mass[i]![j]!*scaleQ;
    for(let c=0;c<m;c++)for(let j=0;j<3;j++)a[n+3*c+j]=-p.contacts[c]!.jacobian[j]![i]!*scaleF;
    return a;
  });
  for(let i=0;i<n;i++){
    const lim=i<r?0:p.torqueLimits[i-r]!;
    const a=dynamics[i]!,denom=Math.max(1,Math.hypot(...a),Math.abs(p.bias[i]!),lim);
    rows.push({a:a.map(v=>v/denom),lo:(-lim-p.bias[i]!)/denom,hi:(lim-p.bias[i]!)/denom});
  }
  for(const t of p.tasks){
    if(t.a.length!==n)throw new RangeError("Invalid acceleration task");
    tasks.push({...t,a:[...t.a.map(v=>v*scaleQ),...Array<number>(3*m).fill(0)]});
  }
  for(let c=0;c<m;c++){
    const contact=p.contacts[c]!,j=contact.jacobian;
    if(j.length!==3||j.some(a=>a.length!==n||!a.every(Number.isFinite))||!Number.isFinite(contact.mu)||contact.mu<0||
      !(contact.maxNormalN>0)||!Number.isFinite(contact.maxNormalN))throw new RangeError("Invalid point contact");
    const z=zeros();z[n+3*c+2]=1;
    rows.push({a:z,lo:0,hi:contact.maxNormalN/scaleF});
    for(const sx of [-1,1])for(const sy of [-1,1]){
      const a=zeros();a[n+3*c]=sx;a[n+3*c+1]=sy;a[n+3*c+2]=-contact.mu;
      rows.push({a,lo:-Infinity,hi:0});
    }
    for(let k=0;k<3;k++){
      const a=zeros();for(let d=0;d<n;d++)a[d]=j[k]![d]!*scaleQ;
      tasks.push({a,b:(contact.targetAcceleration?.[k]??0)-(contact.accelerationBias?.[k]??0),weight:5});
      const forceReg=zeros();forceReg[n+3*c+k]=1;tasks.push({a:forceReg,b:0,weight:.002});
    }
  }
  for(let d=0;d<n;d++){const a=zeros();a[d]=1;tasks.push({a,b:0,weight:.002});}
  const solver=constrainedLeastSquares({n:total,tasks,constraints:rows,rho:20,regularization:1e-6,iterations:3000,tolerance:2e-6});
  const acceleration=solver.x.slice(0,n).map(x=>x*scaleQ);
  const forces=Array.from({length:m},(_,c)=>solver.x.slice(n+3*c,n+3*c+3).map(x=>x*scaleF));
  const effort=dynamics.map((a,i)=>dot(a,solver.x)+p.bias[i]!);
  const rootResidual=effort.slice(0,r),torques=effort.slice(r);
  const maxRootResidual=rootResidual.reduce((v,x)=>Math.max(v,Math.abs(x)),0);
  const maxTorqueViolation=torques.reduce((v,x,i)=>Math.max(v,Math.abs(x)-p.torqueLimits[i]!),0);
  let maxFrictionViolation=0,contactAccelerationResidual=0;
  for(let c=0;c<m;c++){
    const f=forces[c]!,ct=p.contacts[c]!;
    maxFrictionViolation=Math.max(maxFrictionViolation,-f[2]!,f[2]!-ct.maxNormalN,Math.abs(f[0]!)+Math.abs(f[1]!)-ct.mu*f[2]!);
    for(let k=0;k<3;k++)contactAccelerationResidual=Math.max(contactAccelerationResidual,Math.abs(dot(ct.jacobian[k]!,acceleration)+(ct.accelerationBias?.[k]??0)-(ct.targetAcceleration?.[k]??0)));
  }
  return {acceleration,torques,forces,rootResidual,maxRootResidual,maxTorqueViolation,maxFrictionViolation,contactAccelerationResidual,solver,
    feasible:solver.converged&&maxRootResidual<.05&&maxTorqueViolation<.05&&maxFrictionViolation<.05};
}
