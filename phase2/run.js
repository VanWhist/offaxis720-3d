'use strict';
const C=require('./core.js'), Mo=require('./model.js');
const R2D=180/Math.PI;
const fmt=(v,n)=>Number(v).toFixed(n===undefined?2:n);
const pad=(s,n)=>String(s).padStart(n);

function mk(over){ return C.buildSkeleton(Object.assign({},Mo.P_BASE,over||{})); }

// 離地姿勢を (tilt, phi) から作る（index.html の rootQuatFrom と同じ構成）
function takeoffQuat(tiltDeg, phi){
  const t=tiltDeg/180*Math.PI;
  const Hh=[0,0,1], e1=[0,1,0], e2=C.vcrs(Hh,e1);            // e2 = [-1,0,0]
  const L=[Math.cos(t)*(Math.cos(phi)*e1[0]+Math.sin(phi)*e2[0])+Math.sin(t)*Hh[0],
           Math.cos(t)*(Math.cos(phi)*e1[1]+Math.sin(phi)*e2[1])+Math.sin(t)*Hh[1],
           Math.cos(t)*(Math.cos(phi)*e1[2]+Math.sin(phi)*e2[2])+Math.sin(t)*Hh[2]];
  const Ln=C.vunit(L);
  let z0=[Hh[0]-C.vdot(Hh,Ln)*Ln[0], Hh[1]-C.vdot(Hh,Ln)*Ln[1], Hh[2]-C.vdot(Hh,Ln)*Ln[2]];
  z0=C.vunit(z0);
  const x0=C.vunit(C.vcrs(Ln,z0));
  const R=[[x0[0],Ln[0],z0[0]],[x0[1],Ln[1],z0[1]],[x0[2],Ln[2],z0[2]]];
  const tr=R[0][0]+R[1][1]+R[2][2];
  let q;
  if(tr>0){const s=Math.sqrt(tr+1)*2;q=[(R[2][1]-R[1][2])/s,(R[0][2]-R[2][0])/s,(R[1][0]-R[0][1])/s,0.25*s];}
  else if(R[0][0]>R[1][1]&&R[0][0]>R[2][2]){const s=Math.sqrt(1+R[0][0]-R[1][1]-R[2][2])*2;
    q=[0.25*s,(R[0][1]+R[1][0])/s,(R[0][2]+R[2][0])/s,(R[2][1]-R[1][2])/s];}
  else if(R[1][1]>R[2][2]){const s=Math.sqrt(1+R[1][1]-R[0][0]-R[2][2])*2;
    q=[(R[0][1]+R[1][0])/s,0.25*s,(R[1][2]+R[2][1])/s,(R[0][2]-R[2][0])/s];}
  else {const s=Math.sqrt(1+R[2][2]-R[0][0]-R[1][1])*2;
    q=[(R[0][2]+R[2][0])/s,(R[1][2]+R[2][1])/s,0.25*s,(R[1][0]-R[0][1])/s];}
  const n=Math.hypot(q[0],q[1],q[2],q[3]); return q.map(v=>v/n);
}

// |H0| を「1.65秒で宙返りちょうど360°」に合わせる（フィットするのはこのスカラー1個だけ）
function fitH0(sk,ang,opt){
  opt=opt||{};
  const T=Mo.AIRTIME, dt=opt.dt||1/1500, q0=opt.q0||Mo.Q0;
  const run=h=>{
    const tr=C.integrate(sk,ang,{H0:[0,0,h],q0:q0,T:T,dt:dt,sample:15});
    return {an:C.analyze(tr,Mo.H0_DIR)};
  };
  let lo=1, hi=400;
  for(let i=0;i<44;i++){
    const mid=(lo+hi)/2;
    if(run(mid).an.slice(-1)[0].phiDeg < 360) lo=mid; else hi=mid;
    if(hi-lo<1e-4) break;
  }
  const h=(lo+hi)/2, r=run(h);
  return {H0mag:h, an:r.an};
}

function summarize(an){
  const idx=f=>{let b=0;for(let i=0;i<an.length;i++) if(Math.abs(an[i].t-f*Mo.AIRTIME)<Math.abs(an[b].t-f*Mo.AIRTIME)) b=i; return b;};
  const pick=f=>an[idx(f)];
  let psi=0;
  for(let i=1;i<an.length;i++) psi += (an[i].wb[1]+an[i-1].wb[1])/2*(an[i].t-an[i-1].t);
  return {tiltMaxDeg:Math.max.apply(null,an.map(f=>f.tiltDeg)),
          tilt0:an[0].tiltDeg, twistTotalDeg:psi*R2D,
          phi27:pick(0.27).phiDeg, phi55:pick(0.55).phiDeg, phi76:pick(0.76).phiDeg,
          phiEnd:an[an.length-1].phiDeg,
          t25:pick(0.25).tiltDeg, t41:pick(0.41).tiltDeg, t60:pick(0.60).tiltDeg,
          tEnd:an[an.length-1].tiltDeg};
}

console.log('==============================================================');
console.log(' Phase 2 物理モード — Back Full のみ / コンソール検証');
console.log(' anthropometry: ' + C.DELEVA_M.ref.table + '（基準体 73.0kg / 1.741m）');
console.log(' 状態: {trick:bf, tendency:0.80, exit:0.66, shape:0, preset:A}');
console.log(' 物理量はすべて「仮」（物理パラメータ_Phase2_20260819.md）');
console.log(' 離地姿勢: Rz(24.4083°)＝矢状面内で後傾のみ。tilt=0 から始める');
console.log('==============================================================');

// ================= Step 1 =================
console.log('\n########## Step 1: 非対称な腕は tilt を生むか ##########');
console.log(' 初期 tilt=0 / H0 は水平(+Z)。腕が対称なら tilt は 0 のまま、');
console.log(' 非対称なら tilt が立つ、が期待される挙動。');
for(const pole of [0,0.25]){
  const sk=mk({poleMassKg:pole});
  console.log('\n--- ポール ' + (pole===0?'無し（下限値）':'0.25kg（仮）') + ' ---');
  for(const mode of ['frozen','symmetric','measured']){
    const f=fitH0(sk,Mo.makeAngles(mode),{});
    const s=summarize(f.an);
    const jp={frozen:'関節固定        ',symmetric:'左右対称に開閉  ',measured:'実測（左右非対称）'}[mode];
    console.log('  '+jp+' |H0|='+pad(fmt(f.H0mag,2),6)+'  tilt最大='+pad(fmt(s.tiltMaxDeg),6)+
                '°  ひねり合計='+pad(fmt(s.twistTotalDeg,1),8)+'°');
  }
}

// ================= Step 2-3 =================
console.log('\n########## Step 2-3: 全身 — H0固定で1.65秒を積分 / 実測とのズレ ##########');
for(const pole of [0,0.25]){
  const sk=mk({poleMassKg:pole});
  const f=fitH0(sk,Mo.makeAngles('measured'),{});
  const s=summarize(f.an);
  console.log('\n--- ポール ' + (pole===0?'無し（下限値）':'0.25kg（仮）') + ' ---');
  console.log('  フィットした |H0| = '+fmt(f.H0mag,2)+' kg·m²/s （条件: 1.65秒で宙返り360°。フィットはこのスカラー1個だけ）');
  console.log('  宙返り φ   モデル: τ0.27→'+pad(fmt(s.phi27,1),6)+'°  τ0.55→'+pad(fmt(s.phi55,1),6)+'°  τ0.76→'+pad(fmt(s.phi76,1),6)+'°  τ1.00→'+fmt(s.phiEnd,1)+'°');
  console.log('             実測  :        90.0°         180.0°         270.0°         360.0°');
  console.log('             ズレ  : '+pad(fmt(s.phi27-90,1),12)+'° '+pad(fmt(s.phi55-180,1),13)+'° '+pad(fmt(s.phi76-270,1),13)+'°');
  console.log('  tilt: 最大 '+fmt(s.tiltMaxDeg)+'°  (τ0.25→'+fmt(s.t25)+'° τ0.41→'+fmt(s.t41)+'° τ0.60→'+fmt(s.t60)+'° τ1.00→'+fmt(s.tEnd)+'°)');
  console.log('  ★ひねり合計 = '+fmt(s.twistTotalDeg,1)+'°   （実際のバックフルは 360°）');
}

// ================= Step 4 =================
console.log('\n########## Step 4: Yeadonの検算 — 足元の質量を抜くと tilt は出やすくなるか ##########');
for(const pole of [0,0.25]){
  console.log('\n--- ポール ' + (pole===0?'無し（下限値）':'0.25kg（仮）') + ' ---');
  const cfgs=[{n:'装備あり（板+ビン+ブーツ）',o:{}},
              {n:'板+ビンを0',o:{skiMassKg:0,bindingMassKg:0}},
              {n:'足元すべて0',o:{skiMassKg:0,bindingMassKg:0,bootMassKg:0}}];
  let base=null;
  for(const cfg of cfgs){
    const sk=mk(Object.assign({poleMassKg:pole},cfg.o));
    const f=fitH0(sk,Mo.makeAngles('measured'),{});
    const s=summarize(f.an);
    const r0=C.inertiaAndHrel(sk,Mo.makeAngles('measured'),0,{});
    if(!base) base=s.tiltMaxDeg;
    console.log('  '+cfg.n.padEnd(26)+' Iyy(ひねり軸)='+pad(fmt(r0.I[1][1],3),6)+
                '  tilt最大='+pad(fmt(s.tiltMaxDeg),6)+'°  ひねり='+pad(fmt(s.twistTotalDeg,1),8)+'°  '+
                (s.tiltMaxDeg===base?'(基準)':'tilt '+(s.tiltMaxDeg>base?'+':'')+fmt((s.tiltMaxDeg/base-1)*100,1)+'%'));
  }
}

// ================= Step 5 =================
console.log('\n########## Step 5（追加）: 離地時に tilt が有ったら、ひねりはいくつ出るか ##########');
console.log(' 空中の腕だけでは 360° に届かないので、離地時 tilt を振って必要量を見る。');
console.log(' これは予測であってフィットではない（|H0| だけは各条件で360°に合わせ直す）。');
for(const pole of [0,0.25]){
  console.log('\n--- ポール ' + (pole===0?'無し（下限値）':'0.25kg（仮）') + ' ---');
  const sk=mk({poleMassKg:pole});
  console.log('   離地tilt |  |H0|  | tilt最大 | ひねり合計');
  for(const t0 of [0,2,5,10,15,20,25,30]){
    const q0=takeoffQuat(t0,Mo.PHI0);
    const f=fitH0(sk,Mo.makeAngles('measured'),{q0:q0});
    const s=summarize(f.an);
    console.log('   '+pad(fmt(t0,0),6)+'° | '+pad(fmt(f.H0mag,1),5)+' | '+pad(fmt(s.tiltMaxDeg),7)+'° | '+pad(fmt(s.twistTotalDeg,1),8)+'°');
  }
}

// ================= 参考 =================
console.log('\n########## 参考: 慣性モーメントの内訳（離地時姿勢, kg·m²） ##########');
for(const cfg of [{n:'身体のみ',o:{skiMassKg:0,bindingMassKg:0,bootMassKg:0,poleMassKg:0}},
                  {n:'＋足元装備',o:{poleMassKg:0}},
                  {n:'＋足元装備＋ポール0.25',o:{poleMassKg:0.25}}]){
  const sk=mk(cfg.o);
  const r=C.inertiaAndHrel(sk,Mo.makeAngles('measured'),0,{});
  console.log('  '+cfg.n.padEnd(24)+' Iyy(ひねり)='+pad(fmt(r.I[1][1],3),6)+
              '  Izz(宙返り)='+pad(fmt(r.I[2][2],3),6)+'  総質量='+fmt(r.Mtot,2)+'kg');
}
