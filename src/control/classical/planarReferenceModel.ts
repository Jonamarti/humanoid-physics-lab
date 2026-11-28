/**
 * Planar reference geometry, numerical data adapted from Michael Firmin's
 * SimbiconJS static/human.js (ddcb94844d2722e117aa2f07569d0fa356a72776).
 * MIT notice retained below. No Coach or Ammo source included.
 * Model uses Y-up source data converted to HPL X-forward, Y-left, Z-up.
 *
 * The MIT License (MIT)
 * 
 * Copyright (c) 2015 Michael Firmin
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
type V3=readonly [number,number,number];
interface Part {name:string;mass:number;position:V3;shape:"BOX"|"SPHERE"|"CAPSULE";size:readonly number[]}
interface Joint {name:string;parent:string;child:string;anchor:V3;range:readonly [number,number]}
export const referenceParts:readonly Part[]=[
  {"name":"head","mass":4.5,"position":[0,1.728,0],"shape":"CAPSULE","size":[0.09,0.11]},
  {"name":"lArm","mass":2,"position":[0,1.347,-0.1],"shape":"CAPSULE","size":[0.0385,0.303]},
  {"name":"lFoot","mass":1,"position":[0.065,0.025,-0.1],"shape":"BOX","size":[0.18,0.05,0.07]},
  {"name":"lForearm","mass":1,"position":[0,1.058,-0.1],"shape":"CAPSULE","size":[0.024,0.232]},
  {"name":"lHand","mass":0.5,"position":[0,0.863,-0.1],"shape":"BOX","size":[0.044,0.16,0.1]},
  {"name":"lShin","mass":3,"position":[0,0.278,-0.1],"shape":"CAPSULE","size":[0.0335,0.433]},
  {"name":"lThigh","mass":7,"position":[0,0.719,-0.1],"shape":"CAPSULE","size":[0.05,0.4]},
  {"name":"lTorso","mass":7,"position":[0,1.044,0],"shape":"SPHERE","size":[0.065]},
  {"name":"neck","mass":0.5,"position":[0,1.574,0],"shape":"SPHERE","size":[0.04]},
  {"name":"pelvis","mass":6.5,"position":[0,0.95,0],"shape":"SPHERE","size":[0.06]},
  {"name":"rArm","mass":2,"position":[0,1.347,0.1],"shape":"CAPSULE","size":[0.0385,0.303]},
  {"name":"rFoot","mass":1,"position":[0.065,0.025,0.1],"shape":"BOX","size":[0.18,0.05,0.07]},
  {"name":"rForearm","mass":1,"position":[0,1.058,0.1],"shape":"CAPSULE","size":[0.024,0.232]},
  {"name":"rHand","mass":0.5,"position":[0,0.863,0.1],"shape":"BOX","size":[0.044,0.16,0.1]},
  {"name":"rShin","mass":3,"position":[0,0.278,0.1],"shape":"CAPSULE","size":[0.0335,0.433]},
  {"name":"rThigh","mass":7,"position":[0,0.719,0.1],"shape":"CAPSULE","size":[0.05,0.4]},
  {"name":"uTorso","mass":16,"position":[0,1.327,0],"shape":"CAPSULE","size":[0.085,0.3]},
];
export const referenceJoints:readonly Joint[]=[
  {"name":"lAnkle","parent":"lShin","child":"lFoot","anchor":[0,0.042,-0.1],"range":[-60,60]},
  {"name":"lElbow","parent":"lArm","child":"lForearm","anchor":[0,1.174,-0.1],"range":[-157.5,0]},
  {"name":"lHip","parent":"pelvis","child":"lThigh","anchor":[0,0.923,0],"range":[-135,45]},
  {"name":"lKnee","parent":"lThigh","child":"lShin","anchor":[0,0.493,-0.1],"range":[0,157.5]},
  {"name":"lShoulder","parent":"uTorso","child":"lArm","anchor":[0,1.475,0],"range":[-179.999,90]},
  {"name":"lTorso2uTorso","parent":"lTorso","child":"uTorso","anchor":[0,1.097,0],"range":[-22.5,22.5]},
  {"name":"lWrist","parent":"lForearm","child":"lHand","anchor":[-0.004,0.924,-0.1],"range":[-90,90]},
  {"name":"neck2head","parent":"neck","child":"head","anchor":[0,1.588,0],"range":[-45,45]},
  {"name":"rAnkle","parent":"rShin","child":"rFoot","anchor":[0,0.042,0.1],"range":[-60,60]},
  {"name":"rElbow","parent":"rArm","child":"rForearm","anchor":[0,1.174,0.1],"range":[-157.5,0]},
  {"name":"rHip","parent":"pelvis","child":"rThigh","anchor":[0,0.923,0],"range":[-135,45]},
  {"name":"rKnee","parent":"rThigh","child":"rShin","anchor":[0,0.493,0.1],"range":[0,157.5]},
  {"name":"rShoulder","parent":"uTorso","child":"rArm","anchor":[0,1.475,0],"range":[-179.999,90]},
  {"name":"rWrist","parent":"rForearm","child":"rHand","anchor":[-0.004,0.924,0.1],"range":[-90,90]},
  {"name":"uTorso2neck","parent":"uTorso","child":"neck","anchor":[0,1.559,0],"range":[0,0]},
  {"name":"waist","parent":"pelvis","child":"lTorso","anchor":[0,0.995,0],"range":[-22.5,22.5]},
];
export const referenceDt=.00025;
export const referenceTorqueLimit=370;
const convert=(v:V3):V3=>[v[0],-v[2],v[1]];
const sub=(a:V3,b:V3):V3=>[a[0]-b[0],a[1]-b[1],a[2]-b[2]];
const str=(v:readonly number[]):string=>v.join(" ");
/** Planar root by two slides and one pitch hinge. All body hinges are +Y:
 * this corresponds to negative old Z rotation of child relative to parent,
 * the sign of the original Bullet hinge angle and applied internal moment.
 * No self collisions, as in the upstream character collision mask.
 * MuJoCo inertia from geometry, contact friction .25, solver and zero joint
 * damping/armature are explicit approximations, not measured Bullet values.
 */
export type ReferenceSpace="planar"|"free3d"|"free3d-roll"|"free3d-yaw"|"free3d-wbc";
export function planarModel(dt=referenceDt,space:ReferenceSpace="planar"):string {
  if(!(dt>0)||!Number.isFinite(dt))throw new RangeError("Invalid timestep");
  function body(name:string,parent:Part|null,edge:Joint|null):string {
    const p=referenceParts.find(x=>x.name===name);
    if(!p)throw new Error("Missing segment "+name);
    const pos=convert(parent?sub(p.position,parent.position):p.position);
    let joint:string;
    if(!edge)joint=space!=="planar"?'<freejoint name="root"/>':'<joint name="root_x" type="slide" axis="1 0 0"/><joint name="root_z" type="slide" axis="0 0 1"/><joint name="root_pitch" axis="0 1 0"/>';
    else {
      const [lo,hi]=edge.range;
      // Original locked neck hinge has zero width; use a declared micro-range
      // since MuJoCo requires lo < hi for limited hinges.
      const range=lo===hi?[-1e-6,1e-6]:[lo*Math.PI/180,hi*Math.PI/180];
      // The planar reference has coincident left/right hip and shoulder
      // anchors. A 3D anatomical model must separate the sockets laterally.
      // Keep that change isolated from all previously verified baselines.
      const sourceAnchor:V3=space==="free3d-wbc"&&/^[lr](Hip|Shoulder)$/.test(edge.name)?
        [edge.anchor[0],edge.anchor[1],edge.name.startsWith("l")?-.1:.1]:edge.anchor;
      const anchor=str(convert(sub(sourceAnchor,p.position)));
      joint='<joint name="'+edge.name+'" axis="0 1 0" pos="'+anchor+'" limited="true" range="'+str(range)+'"/>';
      if((space==="free3d-roll"||space==="free3d-wbc")&&/^[lr](Hip|Ankle)$/.test(edge.name)){
        // Preserve the sagittal anchor. Hip roll is proximal to hip pitch;
        // ankle roll is distal to ankle pitch. Only new hinges have damping
        // and armature; the original planar/sagittal cases are unchanged.
        const lateral='<joint name="'+edge.name+'Roll" axis="1 0 0" pos="'+anchor+'" limited="true" range="-.5 .5" damping="1" armature=".02"/>';
        joint=edge.name.endsWith("Hip")?lateral+joint:joint+lateral;
      }
      if((space==="free3d-yaw"||space==="free3d-wbc")&&/^[lr]Hip$/.test(edge.name)){
        // Keep the original anchor and pitch axis, but allow actual
        // proximal axial rotation. No yaw lock at the root or feet.
        joint='<joint name="'+edge.name+'Yaw" axis="0 0 1" pos="'+anchor+'" limited="true" range="-.5 .5" damping="1" armature=".02"/>'+joint;
      }
    }
    const size=p.shape==="BOX"?[p.size[0]!/2,p.size[2]!/2,p.size[1]!/2]:
      p.shape==="CAPSULE"?[p.size[0]!,p.size[1]!/2]:[p.size[0]!];
    const geom='<geom name="'+name+'_geom" type="'+p.shape.toLowerCase()+'" size="'+str(size)+'" mass="'+p.mass+'"/>';
    return '<body name="'+name+'" pos="'+str(pos)+'">'+joint+geom+
      referenceJoints.filter(j=>j.parent===name).map(j=>body(j.child,p,j)).join("")+'</body>';
  }
  return '<mujoco model="'+(space==="planar"?"planar_reference":space==="free3d"?"free3d_sagittal_baseline":space==="free3d-yaw"?"free3d_heading_reference":space==="free3d-wbc"?"free3d_whole_body_reference":"free3d_coronal_reference")+'"><compiler angle="radian" inertiafromgeom="true"/>'+
    '<option timestep="'+dt+'" gravity="0 0 -9.81" integrator="implicitfast" cone="elliptic" iterations="50"/>'+
    '<default><joint damping="0" armature="0"/><geom contype="2" conaffinity="1" friction=".25 .005 .0001" solref=".015 1" solimp=".9 .95 .001"/></default>'+
    '<worldbody><geom name="floor" type="box" pos="0 0 -.5" size="50 .5 .5" contype="1" conaffinity="2"/>'+
    body("pelvis",null,null)+'</worldbody></mujoco>';
}
export function referenceInitialPosition(name:string):V3 {
  const part=referenceParts.find(p=>p.name===name);
  if(!part)throw new Error("Unknown body");
  return convert(part.position);
}
