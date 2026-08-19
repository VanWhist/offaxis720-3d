'use strict';
// 物理モードの結果を (tilt, phi, psi) に分解して、index.html に埋め込む表を作る。
// この3つがあれば index.html の rootQuatFrom() でそのまま姿勢を復元できる
// （運動学モードと同じ描画経路を使えるので、Renderer に手を入れなくてよい）。
const C=require('./core.js'), Mo=require('./model.js'), takeoffQuat=require('./takeoff.js');
const R2D=180/Math.PI, D2R=Math.PI/180;
const H=[0,0,1], e1=[0,1,0], e2=C.vcrs(H,e1);            // e2 = [-1,0,0]

// index.html の rootQuatFrom と同じ構成（行列で返す）
function rootMatFrom(tilt,phi,psi){
  const ct=Math.cos(tilt), st=Math.sin(tilt);
  const L=C.vunit([0,1,2].map(i=>ct*(Math.cos(phi)*e1[i]+Math.sin(phi)*e2[i])+st*H[i]));
  const d=C.vdot(H,L);
  let z0=C.vunit([H[0]-d*L[0],H[1]-d*L[1],H[2]-d*L[2]]);
  // z0 を L まわりに psi 回す（ロドリゲス）
  const cp=Math.cos(psi), sp=Math.sin(psi), k=C.vcrs(L,z0), kd=C.vdot(L,z0);
  const bz=C.vunit([0,1,2].map(i=>z0[i]*cp + k[i]*sp + L[i]*kd*(1-cp)));
  const bx=C.vunit(C.vcrs(L,bz));
  return [[bx[0],L[0],bz[0]],[bx[1],L[1],bz[1]],[bx[2],L[2],bz[2]]];
}
// 姿勢行列 → (tilt, phi, psi)
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

function fit(sk,ang,q0){
  const run=(h,dt)=>{const tr=C.integrate(sk,ang,{H0:[0,0,h],q0:q0,T:Mo.AIRTIME,dt:dt,sample:15});
    return C.analyze(tr,H).slice(-1)[0].phiDeg;};
  let lo=40,hi=180;
  for(let i=0;i<30;i++){const m=(lo+hi)/2; if(run(m,1/900)<360)lo=m;else hi=m; if(hi-lo<1e-3)break;}
  return (lo+hi)/2;
}

const NS=50;                     // τ を 0..1 の 51点
const TILTS=[0,2.5,5,7.5,10,12.5,15,17.5,20];
const out={};
let worstDeg=0;

for(const pole of [0,0.25]){
  const sk=C.buildSkeleton(Object.assign({},Mo.P_BASE,{poleMassKg:pole}));
  const ang=Mo.makeAngles('measured');
  const key='p'+String(pole).replace('.','');
  out[key]={tilts:TILTS, H0:[], tilt:[], phi:[], psi:[], twistTotal:[], somTotal:[], tiltPeak:[]};
  for(const t0 of TILTS){
    const q0=takeoffQuat(t0,Mo.PHI0);
    const h=fit(sk,ang,q0);
    const dt=1/1500, tr=C.integrate(sk,ang,{H0:[0,0,h],q0:q0,T:Mo.AIRTIME,dt:dt,sample:5});
    const dec=tr.map(f=>decompose(C.qToM(f.q)));
    const phiU=unwrap(dec.map(d=>d.phi)), psiU=unwrap(dec.map(d=>d.psi));
    // 復元誤差（round-trip）
    for(let i=0;i<tr.length;i++){
      const Rr=rootMatFrom(dec[i].tilt,phiU[i],psiU[i]), Ro=C.qToM(tr[i].q);
      let tr3=0; for(let a=0;a<3;a++)for(let b=0;b<3;b++) tr3+=Rr[a][b]*Ro[a][b];
      const ang3=Math.acos(Math.max(-1,Math.min(1,(tr3-1)/2)))*R2D;
      if(ang3>worstDeg) worstDeg=ang3;
    }
    // τ 一様にリサンプル
    const samp=k=>{const tt=k/NS*Mo.AIRTIME; let b=0;
      for(let i=0;i<tr.length;i++) if(Math.abs(tr[i].t-tt)<Math.abs(tr[b].t-tt)) b=i; return b;};
    const T=[],P=[],S=[];
    for(let k=0;k<=NS;k++){const i=samp(k);
      T.push(+(dec[i].tilt*R2D).toFixed(3)); P.push(+(phiU[i]*R2D).toFixed(2)); S.push(+(psiU[i]*R2D).toFixed(2));}
    out[key].H0.push(+h.toFixed(2));
    out[key].tilt.push(T); out[key].phi.push(P); out[key].psi.push(S);
    out[key].twistTotal.push(+S[NS].toFixed(1));
    out[key].somTotal.push(+P[NS].toFixed(1));
    out[key].tiltPeak.push(+Math.max.apply(null,T.map(Math.abs)).toFixed(2));
    console.error(`pole=${pole} tilt0=${t0}  |H0|=${h.toFixed(2)}  som=${P[NS].toFixed(1)}  twist=${S[NS].toFixed(1)}  tiltPeak=${Math.max.apply(null,T.map(Math.abs)).toFixed(2)}`);
  }
}
console.error('\n復元誤差(round-trip) 最大 = '+worstDeg.toExponential(2)+' °');
console.error('（(tilt,phi,psi) から姿勢を復元したときの誤差。小さければ表だけで再現できる）\n');

// index.html 埋め込み用
const j=o=>JSON.stringify(o);
let s='var PHYSICS_TABLE = {\n';
s+='  meta:{ airtimeS:1.65, tauSamples:'+(NS+1)+', takeoffTiltsDeg:'+j(TILTS)+',\n';
s+='         segmentTable:"de Leva (1996) Table 4, male", note:"Phase 2 物理モード。オフラインで解いた結果を (tilt,phi,psi) で保持している" },\n';
for(const k of Object.keys(out)){
  s+='  '+k+':{ H0:'+j(out[k].H0)+', somTotal:'+j(out[k].somTotal)+', twistTotal:'+j(out[k].twistTotal)+', tiltPeak:'+j(out[k].tiltPeak)+',\n';
  s+='    tilt:'+j(out[k].tilt)+',\n    phi:'+j(out[k].phi)+',\n    psi:'+j(out[k].psi)+' },\n';
}
s+='};\n';
console.log(s);
