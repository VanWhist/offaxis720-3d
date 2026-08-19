'use strict';
// 姿勢 ⇔ (tilt, phi, psi) の相互変換。index.html の rootQuatFrom と同じ構成。
// gen_table.js と stepA.js が共有する（定義が2箇所に分かれないように）。
const C=require('./core.js');
const H=[0,0,1], e1=[0,1,0], e2=C.vcrs(H,e1);            // e2 = [-1,0,0]

function rootMatFrom(tilt,phi,psi){
  const ct=Math.cos(tilt), st=Math.sin(tilt);
  const L=C.vunit([0,1,2].map(i=>ct*(Math.cos(phi)*e1[i]+Math.sin(phi)*e2[i])+st*H[i]));
  const d=C.vdot(H,L);
  const z0=C.vunit([H[0]-d*L[0],H[1]-d*L[1],H[2]-d*L[2]]);
  const cp=Math.cos(psi), sp=Math.sin(psi), k=C.vcrs(L,z0), kd=C.vdot(L,z0);
  const bz=C.vunit([0,1,2].map(i=>z0[i]*cp + k[i]*sp + L[i]*kd*(1-cp)));
  const bx=C.vunit(C.vcrs(L,bz));
  return [[bx[0],L[0],bz[0]],[bx[1],L[1],bz[1]],[bx[2],L[2],bz[2]]];
}
function decompose(R){
  const L=[R[0][1],R[1][1],R[2][1]];
  const tilt=Math.asin(Math.max(-1,Math.min(1,C.vdot(L,H))));
  const phi=Math.atan2(C.vdot(L,e2),C.vdot(L,e1));
  const R0=rootMatFrom(tilt,phi,0);
  const bz0=[R0[0][2],R0[1][2],R0[2][2]], bx0=[R0[0][0],R0[1][0],R0[2][0]];
  const bz =[R[0][2], R[1][2], R[2][2]];
  const psi=Math.atan2(C.vdot(bz,bx0), C.vdot(bz,bz0));
  return {tilt:tilt,phi:phi,psi:psi};
}
const unwrap=a=>{const o=[a[0]];for(let i=1;i<a.length;i++){let d=a[i]-a[i-1];
  while(d> Math.PI)d-=2*Math.PI; while(d<-Math.PI)d+=2*Math.PI; o.push(o[i-1]+d);}return o;};

module.exports={H:H,e1:e1,e2:e2,rootMatFrom:rootMatFrom,decompose:decompose,unwrap:unwrap};
