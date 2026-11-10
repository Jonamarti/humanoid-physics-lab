/** Mirror MuJoCo's mju_decodePyramid and mj_contactForce semantics.
 * The WebAssembly binding's output parameter for mj_contactForce does not
 * reliably copy back to JS in the pinned runtime; use solved efc_force.
 * Reference: google-deepmind/mujoco engine_util_misc.c, engine_core_util.c.
 */
export interface DecodedContactForce {
  readonly normalN: number;
  /** Components along mjContact.frame tangent axes 1 and 2 (N). */
  readonly tangentN: readonly [number,number];
  /** 0 means no tangential demand, 1 means the pyramid/ellipse limit. */
  readonly frictionUtilization: number | null;
}

export function decodeSolvedContactForce(input: {
  readonly solverRows: ArrayLike<number>;
  readonly efcAddress: number;
  readonly dim: number;
  readonly pyramidal: boolean;
  readonly friction: ArrayLike<number>;
  readonly adhesionN: number;
}): DecodedContactForce {
  const {solverRows:force,efcAddress:start,dim,pyramidal,friction,adhesionN}=input;
  if (!Number.isSafeInteger(start) || start<0 ||
      !Number.isSafeInteger(dim) || dim<1 || dim>6 ||
      !Number.isFinite(adhesionN) || adhesionN<0) {
    throw new RangeError("Índice o propiedades del contacto MuJoCo inválidos");
  }
  const count=pyramidal && dim>1 ? 2*(dim-1) : dim;
  if (start+count>force.length || friction.length<Math.max(0,dim-1)) {
    throw new RangeError("Restricción de contacto incompleta");
  }
  const values=Array.from({length:count},(_,i)=>force[start+i]!);
  if (!values.every(Number.isFinite) ||
      Array.from({length:Math.max(0,dim-1)},(_,i)=>friction[i]!)
        .some(x=>!Number.isFinite(x)||x<0)) {
    throw new RangeError("Fuerza o fricción no finita");
  }
  const rawNormal=pyramidal && dim>1 ?
    values.reduce((sum,n)=>sum+n,0) : values[0]!;
  const tangent: [number,number]=[0,0];
  for (let i=0;i<Math.min(2,dim-1);i++) {
    tangent[i]=pyramidal ?
      (values[2*i]!-values[2*i+1]!)*friction[i]! :
      values[i+1]!;
  }
  let frictionUtilization:number|null=null;
  if (dim>1 && rawNormal>1e-9) {
    let demand=0;
    let valid=true;
    for(let i=0;i<Math.min(2,dim-1);i++) {
      const mu=friction[i]!;
      const tang=tangent[i];
      if(mu<=0 && Math.abs(tang)>1e-12){valid=false;break;}
      const ratio=mu>0 ? tang/mu : 0;
      demand+=pyramidal ? Math.abs(ratio) : ratio*ratio;
    }
    if(valid) frictionUtilization=(pyramidal ? demand : Math.sqrt(demand)) / rawNormal;
  }
  return {
    normalN:rawNormal-adhesionN,
    tangentN:tangent,
    frictionUtilization,
  };
}
