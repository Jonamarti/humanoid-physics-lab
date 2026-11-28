/** Independent contact-quality audit. It never feeds back into the gait.
 * Keep the original geometric landing metric for paired comparison.
 * Stronger gate: >=20 ms continuous unloaded flight above 1 cm with opposite
 * foot loaded; after geometric touchdown >=40 N for >=40 ms continuously
 * within 150 ms. These are diagnostic thresholds, not full biomechanical validation.
 */
export type FootIndex=0|1;
type Pair=readonly [number,number];
type V3=readonly [number,number,number];
export interface LandingEvent {t:number;foot:FootIndex;geometric:boolean;flightS:number;loadedS:number;accepted:boolean}
export class LandingQualityAudit {
  private flight=0;
  private maxFlight=0;
  private pending:LandingEvent|null=null;
  private streak=0;
  longestConsecutive=0;
  accepted=0;
  readonly events:LandingEvent[]=[];
  constructor(private readonly dt:number) {
    if(!(dt>0)||!Number.isFinite(dt))throw new RangeError("Invalid contact audit dt");
  }
  observe(t:number,loads:Pair,clearance:Pair,swing:FootIndex,fallen:boolean):void {
    if(![t,...loads,...clearance].every(Number.isFinite)||loads.some(x=>x<0))throw new RangeError("Invalid contact audit sample");
    if(this.pending){
      const p=this.pending;
      p.loadedS=loads[p.foot]>=40?p.loadedS+this.dt:0;
      if(fallen||t-p.t>.15+1e-10)this.resolve(false);
      else if(p.loadedS>=.04-1e-10)this.resolve(true);
    }
    const other=1-swing as FootIndex;
    this.flight=!fallen&&clearance[swing]>.01&&loads[swing]<5&&loads[other]>=40?this.flight+this.dt:0;
    this.maxFlight=Math.max(this.maxFlight,this.flight);
  }
  touchdown(t:number,foot:FootIndex,geometric:boolean):void {
    if(this.pending)this.resolve(false);
    this.pending={t,foot,geometric,flightS:this.maxFlight,loadedS:0,accepted:false};
    this.flight=0;this.maxFlight=0;
    if(!geometric||this.pending.flightS<.02-1e-10)this.resolve(false);
  }
  finish():void {if(this.pending)this.resolve(false);}
  private resolve(ok:boolean):void {
    const p=this.pending;
    if(!p)throw new Error("Missing pending landing");
    p.accepted=ok;
    this.events.push({...p});
    this.pending=null;
    this.streak=ok?this.streak+1:0;
    if(ok)this.accepted++;
    this.longestConsecutive=Math.max(this.longestConsecutive,this.streak);
  }
}
export interface SoleTransform {position:V3;rotation:readonly number[]}
/** Estimate displacement of the SAME material point of a rigid sole,
 * rather than drift of the foot center. Convert the currently loaded world
 * contact to the sole frame, then evaluate its position one step earlier.
 * This is a finite-difference estimate using the solver contact midpoint;
 * rolling on a new point is not counted as translation of the foot center.
 */
export function materialPointTravel(previous:SoleTransform,current:SoleTransform,point:V3):number {
  const d=point.map((x,i)=>x-current.position[i]!);
  const local=[0,1,2].map(j=>d.reduce((s,x,i)=>s+x*current.rotation[3*i+j]!,0));
  const old=[0,1].map(i=>previous.position[i]!+local.reduce((s,x,j)=>s+x*previous.rotation[3*i+j]!,0));
  const travel=Math.hypot(point[0]-old[0]!,point[1]-old[1]!);
  if(!Number.isFinite(travel))throw new Error("Nonfinite material contact motion");
  return travel;
}
