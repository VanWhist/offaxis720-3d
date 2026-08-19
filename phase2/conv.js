'use strict';
const C=require('./core.js'), Mo=require('./model.js');
const R2D=180/Math.PI, f=(v,n)=>Number(v).toFixed(n);
const sk=C.buildSkeleton(Object.assign({},Mo.P_BASE));
const ang=Mo.makeAngles('measured');
console.log('積分刻みの収束確認（ポール無し・実測プロファイル・|H0|=84.13 固定）');
console.log('   dt      tilt最大    ひねり合計    phi(1.65s)   H保存誤差');
for(const inv of [750,1500,3000,6000]){
  const dt=1/inv;
  const tr=C.integrate(sk,ang,{H0:[0,0,84.13],q0:Mo.Q0,T:Mo.AIRTIME,dt:dt,sample:Math.round(inv/100)});
  const an=C.analyze(tr,Mo.H0_DIR);
  let psi=0; for(let i=1;i<an.length;i++) psi+=(an[i].wb[1]+an[i-1].wb[1])/2*(an[i].t-an[i-1].t);
  let worst=0;
  for(const fr of tr){ const R=C.qToM(fr.q); const r=C.inertiaAndHrel(sk,ang,fr.t,{});
    const Hw=C.mv(R,[C.mv(r.I,fr.wb)[0]+r.hrel[0],C.mv(r.I,fr.wb)[1]+r.hrel[1],C.mv(r.I,fr.wb)[2]+r.hrel[2]]); worst=Math.max(worst,Math.hypot(Hw[0],Hw[1],Hw[2]-84.13)); }
  console.log(`  1/${String(inv).padEnd(5)} ${f(Math.max(...an.map(a=>a.tiltDeg)),4).padStart(9)}° ${f(psi*R2D,3).padStart(11)}° ${f(an.slice(-1)[0].phiDeg,3).padStart(11)}° ${worst.toExponential(1).padStart(12)}`);
}
