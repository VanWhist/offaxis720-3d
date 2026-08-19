'use strict';
// ===== Step A =====
//  A-1: tilt(t) / ψ(t) に I_L(t)（身体長軸まわり）と I_T(t)（宙返り軸まわり）を併記
//  A-2: ソルバ自身の I(q) を使った解析式との突き合わせ
//         ψ̇_analytic = |H| · sin(tilt) · (1/I_L − 1/I_T)
//  ★合わせに行かない。出たものをそのまま出す。
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

function analyse(poleMass, tilt0Deg, label){
  const sk=C.buildSkeleton(Object.assign({},Mo.P_BASE,{poleMassKg:poleMass}));
  const ang=Mo.makeAngles('measured');
  const q0=takeoffQuat(tilt0Deg, Mo.PHI0);
  const H0mag=fit(sk,ang,q0);
  const dt=1/1500;
  const tr=C.integrate(sk,ang,{H0:[0,0,H0mag],q0:q0,T:Mo.AIRTIME,dt:dt,sample:3});
  const dec=tr.map(x=>D.decompose(C.qToM(x.q)));
  const psiU=D.unwrap(dec.map(d=>d.psi)), phiU=D.unwrap(dec.map(d=>d.phi));

  console.log(`\n================ ${label} ================`);
  console.log(`  離地の傾き ${f(tilt0Deg,1)}° / フィットした |H0| = ${f(H0mag,2)} kg·m²/s（1.65秒で宙返り360°）`);
  console.log('   τ   |  I_L   |  I_T   | 符号つきtilt | ψ̇ソルバ  | ψ̇解析式  |  比   | ω_y     | ψ累積 | |h_rel|/|H|');
  console.log('  -----+--------+--------+-------------+----------+----------+-------+---------+-------+------------');

  const ratios=[];
  for(let k=0;k<=20;k++){
    const tau=k/20, tt=tau*Mo.AIRTIME;
    let b=0; for(let i=0;i<tr.length;i++) if(Math.abs(tr[i].t-tt)<Math.abs(tr[b].t-tt)) b=i;
    const r=C.inertiaAndHrel(sk,ang,tr[b].t,{});
    const IL=r.I[1][1], IT=r.I[2][2];                 // 体幹系: +Y=長軸, +Z=宙返り軸
    const tilt=dec[b].tilt;
    // ソルバ側の ψ̇（Euler分解した ψ の時間微分・中心差分）
    const i0=Math.max(0,b-1), i1=Math.min(tr.length-1,b+1);
    const psidotS=(psiU[i1]-psiU[i0])/(tr[i1].t-tr[i0].t)*R2D;
    const psidotA=H0mag*Math.sin(tilt)*(1/IL-1/IT)*R2D;
    const ratio = Math.abs(psidotA)>1e-9 ? psidotS/psidotA : NaN;
    // 比が意味を持つのは、解析式が十分大きく かつ tilt が0近傍でないところだけ
    if (Math.abs(psidotA)>3 && Math.abs(tilt*R2D)>1.0) ratios.push(ratio);
    const hrelFrac = C.vlen(r.hrel)/H0mag;
    console.log(`  ${f(tau,2)} | ${pad(f(IL,3),6)} | ${pad(f(IT,3),6)} | ${pad(f(tilt*R2D,3),10)}° | ${pad(f(psidotS,1),7)}°/s | ${pad(f(psidotA,1),7)}°/s | ${pad(isFinite(ratio)?f(ratio,3):'—',5)} | ${pad(f(tr[b].wb[1]*R2D,1),6)}°/s | ${pad(f(psiU[b]*R2D,1),5)}° | ${pad(f(hrelFrac*100,1),9)}%`);
  }
  ratios.sort((a,b)=>a-b);
  const mean=ratios.reduce((a,b)=>a+b,0)/ratios.length;
  const med=ratios.length%2 ? ratios[(ratios.length-1)/2] : (ratios[ratios.length/2-1]+ratios[ratios.length/2])/2;
  const sd=Math.sqrt(ratios.reduce((a,b)=>a+(b-mean)*(b-mean),0)/ratios.length);
  console.log(`  → ψ̇ ソルバ/解析式 の比: 中央値 ${f(med,3)} / 平均 ${f(mean,3)} ± ${f(sd,3)}（|ψ̇解析式|>3°/s かつ |tilt|>1° の ${ratios.length} 点）`);
  // I_L / I_T のレンジ
  let ILmin=1e9,ILmax=-1e9,ITmin=1e9,ITmax=-1e9;
  for(let k=0;k<=40;k++){
    const r=C.inertiaAndHrel(sk,ang,k/40*Mo.AIRTIME,{});
    ILmin=Math.min(ILmin,r.I[1][1]); ILmax=Math.max(ILmax,r.I[1][1]);
    ITmin=Math.min(ITmin,r.I[2][2]); ITmax=Math.max(ITmax,r.I[2][2]);
  }
  console.log(`  → I_L は ${f(ILmin,3)} 〜 ${f(ILmax,3)}（+${f((ILmax/ILmin-1)*100,0)}%）/ I_T は ${f(ITmin,3)} 〜 ${f(ITmax,3)}（+${f((ITmax/ITmin-1)*100,0)}%）`);
  return {H0mag:H0mag, mean:mean, sd:sd};
}

console.log('##### Step A: I_L(t) / I_T(t) と解析式との突き合わせ #####');
console.log('状態 {trick:bf, tendency:0.80, exit:0.66, shape:0, preset:A}');
console.log('segmentTable: '+C.DELEVA_M.ref.table);
console.log('I_L = e_y·I·e_y（身体長軸まわり）/ I_T = e_z·I·e_z（宙返り軸まわり）。');
console.log('解析式は軸対称近似なので、非対角成分（離地時 Ixy≈0.87）は無視している。');

analyse(0,    0,    'ポール無し（下限値） / 離地の傾き 0°');
analyse(0,    10.4, 'ポール無し（下限値） / 離地の傾き 10.4°');
analyse(0.25, 0,    'ポール 0.25kg（仮） / 離地の傾き 0°');
analyse(0.25, 10.4, 'ポール 0.25kg（仮） / 離地の傾き 10.4°');
