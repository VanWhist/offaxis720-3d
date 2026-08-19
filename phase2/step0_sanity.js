'use strict';
// Step 0: ソルバ自体の検算（物理として筋が通っているか）
const C=require('./core.js'), Mo=require('./model.js');

const P=Object.assign({},Mo.P_BASE);
const sk=C.buildSkeleton(P);

console.log('=== Step 0: ソルバの検算 ===');
console.log('anthropometry:', C.DELEVA_M.ref.table, '/ 基準体', C.DELEVA_M.ref.massKg,'kg', C.DELEVA_M.ref.statureM,'m');

// --- 質量の突き合わせ ---
const ang=Mo.makeAngles('frozen');
const B=C.poseAt(sk,ang,0,{});
const total=B.reduce((s,b)=>s+b.m,0);
const bodyOnly=total - 2*(sk.bootMass+sk.skiMass) - (sk.poleMass>0?2*sk.poleMass:0);
console.log(`剛体数 ${B.length} / 総質量 ${total.toFixed(3)} kg  （身体ぶん ${bodyOnly.toFixed(3)} kg ← 目標 ${P.athleteMassKg}）`);

// --- 検算1: 関節固定なら剛体。H0を主軸に置けば tilt は動かないはず ---
function rigidTest(axisName, H0dir){
  const tr=C.integrate(sk,ang,{H0:C.vs?0:0}||{});
  return null;
}
// 体幹系での全身慣性（関節固定）
const r0=C.inertiaAndHrel(sk,ang,0,{});
const I0=r0.I;
console.log('\n全身慣性テンソル I (体幹系, kg·m²):');
I0.forEach(row=>console.log('  ',row.map(v=>v.toFixed(4).padStart(9)).join(' ')));
console.log(`  対角: Ixx(前後軸)=${I0[0][0].toFixed(3)}  Iyy(身体長軸=ひねり)=${I0[1][1].toFixed(3)}  Izz(左右軸=宙返り)=${I0[2][2].toFixed(3)}`);
console.log(`  h_rel (関節固定なのでほぼ0): [${r0.hrel.map(v=>v.toExponential(1)).join(', ')}]`);

// --- 検算2: 剛体 + H0 を Izz 主軸方向 → tilt が0のままか ---
const Iz=[0,0,1];
const trZ=C.integrate(sk,ang,{H0:[0,0,26.0],q0:[0,0,0,1],T:1.65,dt:1/2000,sample:100});
const anZ=C.analyze(trZ,[0,0,1]);
const tiltMaxZ=Math.max.apply(null,anZ.map(f=>f.tiltDeg));
console.log(`\n[検算2] 関節固定・H0を宙返り主軸(+Z)に置く → 純粋な宙返りのはず`);
console.log(`  tilt 最大 = ${tiltMaxZ.toExponential(2)} °   （0であるべき）`);
console.log(`  φ(1.65s) = ${anZ[anZ.length-1].phiDeg.toFixed(2)}°`);

// --- 検算3: 角運動量が保存しているか（ワールド系でH0に一致し続けるか）---
let worstH=0;
for(const f of trZ){
  const R=C.qToM(f.q);
  const r=C.inertiaAndHrel(sk,ang,f.t,{});
  const Hbody=C.mv(r.I,f.wb);                    // 体幹系
  const Hworld=C.mv(R,Hbody);                    // ワールド系
  worstH=Math.max(worstH, Math.hypot(Hworld[0]-0,Hworld[1]-0,Hworld[2]-26.0));
}
console.log(`\n[検算3] ワールド系Hの誤差 最大 = ${worstH.toExponential(2)} kg·m²/s   （0であるべき）`);
