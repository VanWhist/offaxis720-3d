'use strict';
// §2の確認: 慣性乗積 Ixy の発生源は板とブーツか？（質量を0にして読む）
const C=require('./core.js'), Mo=require('./model.js');
const f=(v,n)=>Number(v).toFixed(n===undefined?3:n), pad=(s,n)=>String(s).padStart(n);
const ang=Mo.makeAngles('measured');
// τ=0.70（腕は左右対称に戻っている局面）と τ=0.30（腕が非対称の局面）で読む
console.log('慣性乗積の発生源の切り分け（ポール無し）');
console.log('  条件                          |   τ=0.30（腕 非対称）      |   τ=0.70（腕 左右対称）');
console.log('                                |   Ixy     Iyz     I_L      |   Ixy     Iyz     I_L');
for(const cfg of [
   {n:'装備あり（板+ビン+ブーツ）',o:{}},
   {n:'板+ビンを0',               o:{skiMassKg:0,bindingMassKg:0}},
   {n:'ブーツを0',                o:{bootMassKg:0}},
   {n:'足元すべて0',              o:{skiMassKg:0,bindingMassKg:0,bootMassKg:0}}]){
  const sk=C.buildSkeleton(Object.assign({},Mo.P_BASE,{poleMassKg:0},cfg.o));
  const a=C.inertiaAndHrel(sk,ang,0.30*Mo.AIRTIME,{});
  const b=C.inertiaAndHrel(sk,ang,0.70*Mo.AIRTIME,{});
  console.log(`  ${cfg.n.padEnd(26)} | ${pad(f(a.I[0][1]),7)} ${pad(f(a.I[1][2]),7)} ${pad(f(a.I[1][1]),7)}  | ${pad(f(b.I[0][1]),7)} ${pad(f(b.I[1][2]),7)} ${pad(f(b.I[1][1]),7)}`);
}
