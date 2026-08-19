'use strict';
// A-2 の追加診断：解析式が小tilt域で系統的にずれる原因の切り分け。
//  ★訂正（2026-08-19）: 当初「片腕の外転が軸対称を崩す」としたが、これは誤り。
//    ixy_check.js で切り分けた結果：
//      Ixy（≈0.53〜0.60）は **板＋ビンディング** 由来。腕が左右対称のときも同じ大きさで存在する。
//        板+ビンを0にすると 0.602 → −0.069 まで落ちる。ブーツはほとんど寄与しない（0.04程度）。
//      Iyz（≈−0.23〜−0.29）が **腕の非対称** 由来。左右対称に戻ると 0 になる。
//    つまり身体を軸対称でなくしている主因は板であり、腕の非対称は Iyz を足しているだけ。
//  検証: 完全な I を使った ω と、対角成分だけの I を使った ω を並べる。
const C=require('./core.js'), Mo=require('./model.js'), D=require('./decomp.js'), takeoffQuat=require('./takeoff.js');
const R2D=180/Math.PI;
const f=(v,n)=>Number(v).toFixed(n===undefined?2:n), pad=(s,n)=>String(s).padStart(n);

function fit(sk,ang,q0){
  const run=h=>{const tr=C.integrate(sk,ang,{H0:[0,0,h],q0:q0,T:Mo.AIRTIME,dt:1/900,sample:15});
    return C.analyze(tr,D.H).slice(-1)[0].phiDeg;};
  let lo=40,hi=180;
  for(let i=0;i<30;i++){const m=(lo+hi)/2; if(run(m)<360)lo=m;else hi=m; if(hi-lo<1e-3)break;}
  return (lo+hi)/2;
}

function diag(poleMass, tilt0Deg, label){
  const sk=C.buildSkeleton(Object.assign({},Mo.P_BASE,{poleMassKg:poleMass}));
  const ang=Mo.makeAngles('measured');
  const q0=takeoffQuat(tilt0Deg, Mo.PHI0);
  const H0mag=fit(sk,ang,q0);
  const tr=C.integrate(sk,ang,{H0:[0,0,H0mag],q0:q0,T:Mo.AIRTIME,dt:1/1500,sample:3});
  const dec=tr.map(x=>D.decompose(C.qToM(x.q)));

  console.log(`\n=== ${label}（|H0|=${f(H0mag,2)}）===`);
  console.log('   τ   | 符号つきtilt |   Ixy   |   Iyz   | 非対角/I_L | ω_y(完全I) | ω_y(対角のみ) | 比');
  console.log('  -----+-------------+---------+---------+-----------+-----------+--------------+------');
  const rats=[];
  for(let k=4;k<=20;k++){
    const tau=k/20, tt=tau*Mo.AIRTIME;
    let b=0; for(let i=0;i<tr.length;i++) if(Math.abs(tr[i].t-tt)<Math.abs(tr[b].t-tt)) b=i;
    const r=C.inertiaAndHrel(sk,ang,tr[b].t,{});
    const I=r.I, IL=I[1][1];
    const R=C.qToM(tr[b].q);
    const rhs=[0,1,2].map(i=>C.mv(C.mT(R),[0,0,H0mag])[i]-r.hrel[i]);
    const wFull=C.mv(C.minv(I),rhs);
    const Id=[[I[0][0],0,0],[0,I[1][1],0],[0,0,I[2][2]]];
    const wDiag=C.mv(C.minv(Id),rhs);
    const off=Math.max(Math.abs(I[0][1]),Math.abs(I[1][2]),Math.abs(I[0][2]));
    const ratio=Math.abs(wDiag[1])>1e-6 ? wFull[1]/wDiag[1] : NaN;
    if(Math.abs(dec[b].tilt*R2D)>1.0) rats.push(ratio);
    console.log(`  ${f(tau,2)} | ${pad(f(dec[b].tilt*R2D,3),10)}° | ${pad(f(I[0][1],3),7)} | ${pad(f(I[1][2],3),7)} | ${pad(f(off/IL*100,1),8)}% | ${pad(f(wFull[1]*R2D,1),8)}°/s | ${pad(f(wDiag[1]*R2D,1),11)}°/s | ${pad(isFinite(ratio)?f(ratio,3):'—',5)}`);
  }
  const mean=rats.reduce((a,b)=>a+b,0)/rats.length;
  console.log(`  → ω_y(完全I)/ω_y(対角のみ) の平均 = ${f(mean,3)}（|tilt|>1° の ${rats.length} 点）`);
}

console.log('##### A-2 追加診断: 解析式のズレの原因 #####');
console.log('解析式 ψ̇=|H|sin(tilt)(1/I_L−1/I_T) は「慣性テンソルが対角」を仮定している。');
console.log('モーグルでは板が身体を軸対称でなくしているので、その仮定が崩れているかを見る。');
diag(0, 0,    'ポール無し / 離地の傾き 0°');
diag(0, 10.4, 'ポール無し / 離地の傾き 10.4°');
