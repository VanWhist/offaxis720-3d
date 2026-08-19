'use strict';
// アプリ(index.html)と同じ入力を、物理側でもそのまま使うための定義

// ---- PHYSICAL_PARAMS（すべて仮。物理パラメータ_Phase2_20260819.md）----
const P_BASE = {
  athleteHeightM: 1.70,
  athleteMassKg : 65,
  skiLengthM    : 1.72,
  skiMassKg     : 1.65,
  bindingMassKg : 0.875,
  bootMassKg    : 1.75,
  poleLengthM   : 1.05,   // 既存値（未実測）
  poleMassKg    : 0.00,   // 既定は「ポール無し＝下限値」。有りは 0.25 仮 を別途走らせる
  // 肩幅・腰幅は描画モデル由来（実測ではない）
  shoulderHalfM : 0.22,
  hipHalfM      : 0.11
};

// ---- ARM_PROFILES.bf（τドメイン・左右独立）----
const ARM_KEYS = [
  [0.00, 0.22, 0.22, 2.85, -0.12],
  [0.19, 0.24, 0.24, 2.85, -0.12],
  [0.245,1.39, 0.10, 2.85, -0.12],
  [0.415,1.39, 0.10, 2.85, -0.12],
  [0.55, 0.26, 0.26, 2.85, -0.12],
  [0.60, 0.25, 0.25, 2.85, -0.12],
  [0.80, 0.95, 0.95, 2.80, -0.10],
  [1.00, 1.75, 1.75, 2.70, -0.08]
];
const clamp=(v,a,b)=>v<a?a:(v>b?b:v);
const smooth=u=>{u=clamp(u,0,1);return u*u*(3-2*u);};
const lerp=(a,b,u)=>a+(b-a)*u;
function track(keys,col){
  const xs=keys.map(k=>k[0]), ys=keys.map(k=>k[col]);
  return x=>{
    if(x<=xs[0])return ys[0];
    if(x>=xs[xs.length-1])return ys[ys.length-1];
    for(let i=0;i<xs.length-1;i++) if(x<=xs[i+1])
      return lerp(ys[i],ys[i+1],smooth((x-xs[i])/(xs[i+1]-xs[i])));
    return ys[ys.length-1];
  };
}
const AB=track(ARM_KEYS,1), TU=track(ARM_KEYS,2), SW=track(ARM_KEYS,3), EL=track(ARM_KEYS,4);

const AIRTIME = 1.65;

// mode: 'measured' 実測プロファイル / 'symmetric' 左右対称の対照 / 'frozen' 関節固定
function makeAngles(mode, airtime){
  airtime = airtime || AIRTIME;
  return function(t){
    const tau = clamp(t/airtime,0,1);
    let sL, sR;
    if(mode==='frozen'){ sL=sR=AB(0); }
    else if(mode==='symmetric'){ sL=sR=(AB(tau)+TU(tau))/2; }
    else { sL=AB(tau); sR=TU(tau); }            // sideAssignment: 外転側=L（仮置き）
    const swing = mode==='frozen'? SW(0) : SW(tau);
    const elbow = mode==='frozen'? EL(0) : EL(tau);
    // 脚（shape=0・レイアウト。index.html の localQuats と同じ）
    const u = mode==='frozen'?0:clamp(t/0.30,0,1);
    const hip = lerp(0.10,0.06,u), knee = lerp(0.12,0.10,u);
    return {spreadL:sL, spreadR:sR, swing:swing, elbow:elbow,
            hip:hip, knee:knee, ankle:knee*0.35-hip*0.35};
  };
}

// ============================================================
//  B-2: Cork 720 を扱うときの規約（映像が届く前に固定しておく）
//    映像を見てから決めると、仮説検証ではなくフィッティングになるため。
//
//   フィットしてよい : |H0| の「大きさ」だけ。
//                      ただし **Cork 自身の宙返り進行から決める**。
//                      Back Full の値（84〜89 kg·m²/s）を流用しないこと。
//   測定から与える   : q(t)（関節角）、初期姿勢、読めれば H0 の「向き」
//   絶対にフィットしない : tilt(t) と twist(t)  ← これらは出力
//
//   映像から推定した H を「正解」として扱わないこと。微分を伴うのでノイズに弱く、
//   Yeadon の11セグメント法でも相対誤差は約10%と見積もられている。
//   保存性と向きの安定性を見る用途に留める。
//
//  B-5: 比較指標はこの5つに固定する
//    somersault(t) / tilt(t) / twist(t) / ω（L・T・AP成分） / q(t)
// ============================================================

// 離地時の姿勢。
//  ★物理モードでは tilt は「出力」なので、初期姿勢に tilt を入れてはいけない。
//    運動学モデルの離地quaternionは preset A の tilt 22.08° を既に含んでいるので使わない。
//    使うのは「矢状面内で後傾しただけの姿勢」＝ Rz(phi0)、phi0 = 24.4083°（運動学モデルと同じ離地角）
const PHI0 = 0.42600562737583103;                       // rad（= 24.4083°）
const Q0 = [0, 0, 0.2113958, 0.97740054];               // Rz(PHI0)。tilt = 0
const Q0_KINEMATIC = [0.18147171,0.06464204,0.2301895,0.95388795];  // 参考: 運動学モデルの離地姿勢（tilt22.08°入り）
const H0_DIR = [0,0,1];   // bF: ほぼ水平（実測寄り）

module.exports={P_BASE,ARM_KEYS,makeAngles,Q0,Q0_KINEMATIC,PHI0,H0_DIR,AIRTIME,clamp,smooth,lerp};
