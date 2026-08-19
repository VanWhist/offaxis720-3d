'use strict';
// §2: tilt(t) と ψ(t) の時系列。往復動作で打ち消しているのかを見る。
// ★合わせに行かない。出たものをそのまま出す。
const C=require('./core.js'), Mo=require('./model.js');
const R2D=180/Math.PI, f=(v,n)=>Number(v).toFixed(n===undefined?2:n), pad=(s,n)=>String(s).padStart(n);

const H=[0,0,1];
// 符号つき tilt = asin(L̂·Ĥ)（報告してきた tilt は絶対値だったので、符号を出す）
function signedTilt(L){ return Math.asin(Math.max(-1,Math.min(1,C.vdot(C.vunit(L),H))))*R2D; }

function series(poleMass, H0mag, label){
  const sk=C.buildSkeleton(Object.assign({},Mo.P_BASE,{poleMassKg:poleMass}));
  const ang=Mo.makeAngles('measured');
  const tr=C.integrate(sk,ang,{H0:[0,0,H0mag],q0:Mo.Q0,T:Mo.AIRTIME,dt:1/1500,sample:5});
  const an=C.analyze(tr,H);
  // ψ の累積（ω_body の Y 成分を積分）と、その瞬時値
  let psi=0; const rows=[];
  const A=Mo.makeAngles('measured');
  for(let i=0;i<tr.length;i++){
    if(i>0) psi += (tr[i].wb[1]+tr[i-1].wb[1])/2*(tr[i].t-tr[i-1].t);
    rows.push({tau:tr[i].t/Mo.AIRTIME, st:signedTilt(tr[i].L), psi:psi*R2D,
               psidot:tr[i].wb[1]*R2D, phi:an[i].phiDeg,
               spreadL:A(tr[i].t).spreadL*R2D, spreadR:A(tr[i].t).spreadR*R2D});
  }
  console.log(`\n=== ${label} （|H0|=${H0mag}, 離地tilt=0） ===`);
  console.log('   τ  | 腕の左右差 | 符号つきtilt | ひねり速度 | ひねり累積 | 宙返りφ');
  console.log('  ----+-----------+-------------+-----------+-----------+--------');
  for(let k=0;k<=20;k++){
    const tau=k/20;
    let b=0; for(let i=0;i<rows.length;i++) if(Math.abs(rows[i].tau-tau)<Math.abs(rows[b].tau-tau)) b=i;
    const r=rows[b];
    console.log(`  ${f(tau,2)} | ${pad(f(r.spreadL-r.spreadR,1),8)}° | ${pad(f(r.st,3),10)}° | ${pad(f(r.psidot,1),8)}°/s | ${pad(f(r.psi,1),8)}° | ${pad(f(r.phi,1),6)}°`);
  }
  // 行き（tiltが正の側）と帰り（負の側）の寄与を分けて積算
  let posContrib=0, negContrib=0;
  for(let i=1;i<rows.length;i++){
    const d=(rows[i].psidot+rows[i-1].psidot)/2*(rows[i].tau-rows[i-1].tau)*Mo.AIRTIME;
    if(d>0) posContrib+=d; else negContrib+=d;
  }
  console.log(`  → ひねりの内訳: 正方向 +${f(posContrib,1)}°  /  逆方向 ${f(negContrib,1)}°  /  差し引き ${f(posContrib+negContrib,1)}°`);
  console.log(`  → 符号つきtilt の範囲: ${f(Math.min.apply(null,rows.map(r=>r.st)),3)}° 〜 ${f(Math.max.apply(null,rows.map(r=>r.st)),3)}°`);
  return rows;
}

console.log('##### §2: tilt(t) と ψ(t) の時系列 #####');
console.log('状態 {trick:bf, tendency:0.80, exit:0.66, shape:0, preset:A} / 離地tilt=0');
series(0,    84.13, 'ポール無し（下限値）');
series(0.25, 86.97, 'ポール0.25kg（仮）');

// --- 打ち消しの検証: 腕を「開いたまま戻さない」場合と比較 ---
console.log('\n##### 追加検証: 腕を戻さなかったら（開いたまま保持）#####');
console.log('※これは実測ではない。打ち消しが起きているかを確かめるためだけの仮想条件。');
function holdOpen(t){
  const tau=Mo.clamp(t/Mo.AIRTIME,0,1);
  const A=Mo.makeAngles('measured');
  // τ0.415 以降を τ0.415 の値で固定（＝戻さない）
  return A(Math.min(t, 0.415*Mo.AIRTIME));
}
for(const [pole,h] of [[0,84.13],[0.25,86.97]]){
  const sk=C.buildSkeleton(Object.assign({},Mo.P_BASE,{poleMassKg:pole}));
  const tr=C.integrate(sk,holdOpen,{H0:[0,0,h],q0:Mo.Q0,T:Mo.AIRTIME,dt:1/1500,sample:15});
  let psi=0; for(let i=1;i<tr.length;i++) psi+=(tr[i].wb[1]+tr[i-1].wb[1])/2*(tr[i].t-tr[i-1].t);
  const st=tr.map(x=>signedTilt(x.L));
  console.log(`  ポール${pole===0?'無し   ':'0.25kg'}: 符号つきtilt ${f(Math.min.apply(null,st),2)}〜${f(Math.max.apply(null,st),2)}°  ひねり合計 ${f(psi*R2D,1)}°`);
}

// --- §3: 離地tilt sweep で |H0| を再フィットしたか / 宙返り総量は360°か ---
console.log('\n##### §3: 離地tilt sweep の |H0| と宙返り総量 #####');
console.log('報告した sweep は tilt ごとに |H0| を再フィットしている（各条件で1.65秒・360°）。');
console.log('固定した場合と並べる。');
function sweepRow(sk, ang, q0, H0mag){
  const tr=C.integrate(sk,ang,{H0:[0,0,H0mag],q0:q0,T:Mo.AIRTIME,dt:1/1500,sample:15});
  const an=C.analyze(tr,H);
  let psi=0; for(let i=1;i<tr.length;i++) psi+=(tr[i].wb[1]+tr[i-1].wb[1])/2*(tr[i].t-tr[i-1].t);
  return {phi:an[an.length-1].phiDeg, psi:psi*R2D, tiltMax:Math.max.apply(null,an.map(a=>a.tiltDeg))};
}
const takeoffQuat=require('./takeoff.js');
for(const pole of [0,0.25]){
  const sk=C.buildSkeleton(Object.assign({},Mo.P_BASE,{poleMassKg:pole}));
  const ang=Mo.makeAngles('measured');
  const H0fix = pole===0?84.13:86.97;
  console.log(`\n--- ポール ${pole===0?'無し':'0.25kg'} ---`);
  console.log('  離地tilt |  再フィット: |H0| / 宙返り / ひねり  |  |H0|固定: 宙返り / ひねり');
  for(const t0 of [0,5,10,15,20]){
    const q0=takeoffQuat(t0,Mo.PHI0);
    // 再フィット
    let lo=1,hi=400;
    for(let i=0;i<40;i++){const m=(lo+hi)/2; if(sweepRow(sk,ang,q0,m).phi<360) lo=m; else hi=m; if(hi-lo<1e-4)break;}
    const hFit=(lo+hi)/2, rFit=sweepRow(sk,ang,q0,hFit);
    const rFix=sweepRow(sk,ang,q0,H0fix);
    console.log(`  ${pad(t0,6)}°  |  ${pad(f(hFit,2),6)} / ${pad(f(rFit.phi,1),6)}° / ${pad(f(rFit.psi,1),7)}°  |  ${pad(f(rFix.phi,1),6)}° / ${pad(f(rFix.psi,1),7)}°`);
  }
}
