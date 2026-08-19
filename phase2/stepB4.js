'use strict';
// ===== B-4: 感度範囲 =====
// 「ひねり360°に必要な離地の傾き」が、未確定な仮定でどれだけ動くかを1回で出す。
//  ・腕の左右割り当て（sideAssignment）現行 / 入れ替え  ← 一番重要
//  ・ポール 無し / 0.20 / 0.25 / 0.30 kg（未実測）
//  ・ビンディング 0.50 / 0.875 / 1.25 kg（一番粗い量）
// |H0| は条件ごとに1回だけ（離地の傾き10°で）合わせ、同じ条件内では固定する。
// Step A で |H0| の tilt 依存は 1% 程度と分かっているため。宙返り総量も併記して確認する。
const C=require('./core.js'), Mo=require('./model.js'), D=require('./decomp.js'), takeoffQuat=require('./takeoff.js');
const R2D=180/Math.PI;
const f=(v,n)=>Number(v).toFixed(n===undefined?1:n), pad=(s,n)=>String(s).padStart(n);

// 左右割り当てを入れ替えた関節角
function makeAngles(swap){
  const base=Mo.makeAngles('measured');
  if(!swap) return base;
  return function(t){ const a=base(t);
    return {spreadL:a.spreadR, spreadR:a.spreadL, swing:a.swing, elbow:a.elbow,
            hip:a.hip, knee:a.knee, ankle:a.ankle}; };
}

function run(sk,ang,q0,H0mag,dt){
  const tr=C.integrate(sk,ang,{H0:[0,0,H0mag],q0:q0,T:Mo.AIRTIME,dt:dt||1/1500,sample:5});
  const dec=tr.map(x=>D.decompose(C.qToM(x.q)));
  const psi=D.unwrap(dec.map(d=>d.psi)), phi=D.unwrap(dec.map(d=>d.phi));
  return { twist:(psi[psi.length-1]-psi[0])*R2D, som:(phi[phi.length-1]-phi[0])*R2D,
           tiltPeak:Math.max.apply(null,dec.map(d=>Math.abs(d.tilt)))*R2D };
}
function fitH0(sk,ang,q0){
  let lo=40,hi=180;
  for(let i=0;i<26;i++){const m=(lo+hi)/2;
    if(run(sk,ang,q0,m,1/900).som<360) lo=m; else hi=m; if(hi-lo<2e-3)break;}
  return (lo+hi)/2;
}

const TILTS=[5,7.5,10,12.5,15];
function required(label, over, swap){
  const sk=C.buildSkeleton(Object.assign({},Mo.P_BASE,over));
  const ang=makeAngles(swap);
  const H0=fitH0(sk,ang,takeoffQuat(10,Mo.PHI0));
  const tw=[], som=[];
  for(const t0 of TILTS){
    const r=run(sk,ang,takeoffQuat(t0,Mo.PHI0),H0);
    tw.push(Math.abs(r.twist)); som.push(r.som);
  }
  let need=null;
  for(let i=0;i<tw.length-1;i++) if(tw[i]<360 && tw[i+1]>=360)
    need = TILTS[i] + (360-tw[i])/(tw[i+1]-tw[i])*(TILTS[i+1]-TILTS[i]);
  console.log(`  ${label.padEnd(38)} | ${pad(need===null?'—':f(need,2),6)}° | |H0|=${pad(f(H0,2),6)} | 宙返り ${pad(f(Math.min.apply(null,som)),5)}〜${pad(f(Math.max.apply(null,som)),5)}° | ひねり ${tw.map(v=>pad(f(v,0),4)).join(' ')}`);
  return need;
}

console.log('##### B-4: 感度範囲 — ひねり360°に必要な離地の傾き #####');
console.log('状態 {trick:bf, tendency:0.80, exit:0.66, shape:0}  /  '+C.DELEVA_M.ref.table);
console.log('|H0| は条件ごとに離地の傾き10°で1回合わせ、条件内は固定。宙返り総量を併記して確認する。');
console.log('ひねりの列は 離地の傾き '+TILTS.join(' / ')+'° での値[°]。\n');
console.log('  条件                                   | 必要傾き | |H0|         | 宙返り総量        | ひねり（傾き別）');
console.log('  ---------------------------------------+---------+--------------+-------------------+-------------------------');

const base = required('現行の左右割当／ポール無／ビン0.875', {poleMassKg:0}, false);
const swap = required('★左右入れ替え／ポール無／ビン0.875', {poleMassKg:0}, true);

console.log('');
for(const p of [0.20,0.25,0.30])
  required(`ポール ${p.toFixed(2)}kg（左右現行・ビン0.875）`, {poleMassKg:p}, false);

console.log('');
for(const b of [0.50,1.25])
  required(`ビンディング ${b.toFixed(3)}kg（ポール無）`, {poleMassKg:0, bindingMassKg:b}, false);

console.log('\n  → 左右入れ替えによる差: ' + (base!==null&&swap!==null ? f(swap-base,2)+'°（'+f((swap/base-1)*100,1)+'%）' : '算出できず'));
