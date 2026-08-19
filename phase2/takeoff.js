'use strict';
const C=require('./core.js');
// 離地姿勢を (tilt, phi) から作る（index.html の rootQuatFrom と同じ構成）
module.exports=function takeoffQuat(tiltDeg, phi){
  const t=tiltDeg/180*Math.PI;
  const Hh=[0,0,1], e1=[0,1,0], e2=C.vcrs(Hh,e1);
  const L=[0,1,2].map(i=>Math.cos(t)*(Math.cos(phi)*e1[i]+Math.sin(phi)*e2[i])+Math.sin(t)*Hh[i]);
  const Ln=C.vunit(L);
  const d=C.vdot(Hh,Ln);
  const z0=C.vunit([Hh[0]-d*Ln[0],Hh[1]-d*Ln[1],Hh[2]-d*Ln[2]]);
  const x0=C.vunit(C.vcrs(Ln,z0));
  const R=[[x0[0],Ln[0],z0[0]],[x0[1],Ln[1],z0[1]],[x0[2],Ln[2],z0[2]]];
  const tr=R[0][0]+R[1][1]+R[2][2];
  let q;
  if(tr>0){const s=Math.sqrt(tr+1)*2;q=[(R[2][1]-R[1][2])/s,(R[0][2]-R[2][0])/s,(R[1][0]-R[0][1])/s,0.25*s];}
  else if(R[0][0]>R[1][1]&&R[0][0]>R[2][2]){const s=Math.sqrt(1+R[0][0]-R[1][1]-R[2][2])*2;
    q=[0.25*s,(R[0][1]+R[1][0])/s,(R[0][2]+R[2][0])/s,(R[2][1]-R[1][2])/s];}
  else if(R[1][1]>R[2][2]){const s=Math.sqrt(1+R[1][1]-R[0][0]-R[2][2])*2;
    q=[(R[0][1]+R[1][0])/s,0.25*s,(R[1][2]+R[2][1])/s,(R[0][2]-R[2][0])/s];}
  else {const s=Math.sqrt(1+R[2][2]-R[0][0]-R[1][1])*2;
    q=[(R[0][2]+R[2][0])/s,(R[1][2]+R[2][1])/s,0.25*s,(R[1][0]-R[0][1])/s];}
  const n=Math.hypot(q[0],q[1],q[2],q[3]); return q.map(v=>v/n);
};
