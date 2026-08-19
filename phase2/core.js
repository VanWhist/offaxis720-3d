// ============================================================
//  Phase 2 物理モード コア（依存なし・bunでもブラウザでも同じ結果）
//  H0固定・関節角入力から ω を解いて姿勢を積分する縮約モデル
//
//  ★モデル名: arm-only-reduced-v1  （2026-08-19 凍結）
//    このファイルの剛体構成・骨格・関節の入れ方は **これ以上変更しない**。
//    理由: コーク720の映像を見てからモデルを変えると、仮説検証ではなく
//          フィッティングになってしまうため。
//    6剛体版など次のモデルは **別ファイル・別名（v2）** で作ること。
//    バグ修正はしてよいが、その場合は MODEL_VERSION を上げて理由を残す。
// ============================================================
'use strict';
const MODEL_VERSION='arm-only-reduced-v1';   // 凍結。構成を変えるときは別ファイルにする

// ---- ベクトル / 3x3行列 / quaternion ----
const vadd=(a,b)=>[a[0]+b[0],a[1]+b[1],a[2]+b[2]];
const vsub=(a,b)=>[a[0]-b[0],a[1]-b[1],a[2]-b[2]];
const vs  =(a,s)=>[a[0]*s,a[1]*s,a[2]*s];
const vdot=(a,b)=>a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
const vcrs=(a,b)=>[a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
const vlen=a=>Math.sqrt(vdot(a,a));
const vunit=a=>{const n=vlen(a);return n<1e-12?[0,0,0]:vs(a,1/n);};

const M3=(a,b,c,d,e,f,g,h,i)=>[[a,b,c],[d,e,f],[g,h,i]];
const mI=()=>M3(1,0,0,0,1,0,0,0,1);
function mmul(A,B){const C=[[0,0,0],[0,0,0],[0,0,0]];
  for(let i=0;i<3;i++)for(let j=0;j<3;j++){let s=0;for(let k=0;k<3;k++)s+=A[i][k]*B[k][j];C[i][j]=s;}return C;}
function mT(A){return M3(A[0][0],A[1][0],A[2][0],A[0][1],A[1][1],A[2][1],A[0][2],A[1][2],A[2][2]);}
function mv(A,x){return [A[0][0]*x[0]+A[0][1]*x[1]+A[0][2]*x[2],
                         A[1][0]*x[0]+A[1][1]*x[1]+A[1][2]*x[2],
                         A[2][0]*x[0]+A[2][1]*x[1]+A[2][2]*x[2]];}
function madd(A,B){const C=[[0,0,0],[0,0,0],[0,0,0]];
  for(let i=0;i<3;i++)for(let j=0;j<3;j++)C[i][j]=A[i][j]+B[i][j];return C;}
function msc(A,s){const C=[[0,0,0],[0,0,0],[0,0,0]];
  for(let i=0;i<3;i++)for(let j=0;j<3;j++)C[i][j]=A[i][j]*s;return C;}
function minv(A){
  const a=A[0][0],b=A[0][1],c=A[0][2],d=A[1][0],e=A[1][1],f=A[1][2],g=A[2][0],h=A[2][1],i=A[2][2];
  const det=a*(e*i-f*h)-b*(d*i-f*g)+c*(d*h-e*g);
  if(Math.abs(det)<1e-18) throw new Error('singular inertia tensor');
  const s=1/det;
  return M3((e*i-f*h)*s,(c*h-b*i)*s,(b*f-c*e)*s,
            (f*g-d*i)*s,(a*i-c*g)*s,(c*d-a*f)*s,
            (d*h-e*g)*s,(b*g-a*h)*s,(a*e-b*d)*s);
}
const rotX=t=>M3(1,0,0, 0,Math.cos(t),-Math.sin(t), 0,Math.sin(t),Math.cos(t));
const rotY=t=>M3(Math.cos(t),0,Math.sin(t), 0,1,0, -Math.sin(t),0,Math.cos(t));
const rotZ=t=>M3(Math.cos(t),-Math.sin(t),0, Math.sin(t),Math.cos(t),0, 0,0,1);
// three.js の Euler 'XYZ' と同じ合成（q = qx*qy*qz → R = Rx·Ry·Rz）
const eulerXYZ=(x,y,z)=>mmul(rotX(x),mmul(rotY(y),rotZ(z)));

// quaternion [x,y,z,w]
function qToM(q){
  const x=q[0],y=q[1],z=q[2],w=q[3];
  return M3(1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w),
            2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w),
            2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y));
}
const qnorm=q=>{const n=Math.hypot(q[0],q[1],q[2],q[3]);return q.map(v=>v/n);};
// q̇ = 0.5 * q ⊗ (ω_body,0)
function qDot(q,wb){
  const x=q[0],y=q[1],z=q[2],w=q[3],a=wb[0],b=wb[1],c=wb[2];
  return [0.5*( w*a + y*c - z*b),
          0.5*( w*b + z*a - x*c),
          0.5*( w*c + x*b - y*a),
          0.5*(-x*a - y*b - z*c)];
}

// ============================================================
//  慣性: de Leva (1996) Table 4, MALE
//    出典: de Leva P. Adjustments to Zatsiorsky-Seluyanov segment
//          inertia parameters. J Biomech 29(9):1223-1230, 1996.
//    基準体: 男性 体重 73.0 kg / 身長 1.741 m
//    mass = 体重比[%], len = セグメント長[mm], cm = 近位(頭側)からのCM位置[%],
//    rSag/rTra/rLon = 矢状/横/長軸まわりの慣性半径[セグメント長%]
//    ※Winter Table 4.1 は C of G / Proximal / Distal の3列だが、これは
//      「同じ軸まわりの基準点違い」（平行軸定理で一致することを確認済み）で、
//      長軸まわりの値が無い。ひねりを扱う本アプリでは使えないため de Leva を採用。
// ============================================================
const DELEVA_M = {
  ref:{ massKg:73.0, statureM:1.741, sex:'male', table:'de Leva (1996) Table 4, male' },
  head    :{mass:6.94, len:203.3, cm:59.76, rSag:36.2, rTra:37.6, rLon:31.2, longAxis:'y'},
  trunk   :{mass:43.46,len:531.9, cm:44.86, rSag:37.2, rTra:34.7, rLon:19.1, longAxis:'y'},
  upperArm:{mass:2.71, len:281.7, cm:57.72, rSag:28.5, rTra:26.9, rLon:15.8, longAxis:'y'},
  forearm :{mass:1.62, len:268.9, cm:45.74, rSag:27.6, rTra:26.5, rLon:12.1, longAxis:'y'},
  hand    :{mass:0.61, len: 86.2, cm:79.00, rSag:62.8, rTra:51.3, rLon:40.1, longAxis:'y'},
  thigh   :{mass:14.16,len:422.2, cm:40.95, rSag:32.9, rTra:32.9, rLon:14.9, longAxis:'y'},
  shank   :{mass:4.33, len:434.0, cm:44.59, rSag:25.5, rTra:24.9, rLon:10.3, longAxis:'y'},
  foot    :{mass:1.37, len:258.1, cm:44.15, rSag:25.7, rTra:24.5, rLon:12.4, longAxis:'x'}
};

// セグメントの主慣性（自身のCMまわり・ローカル系）
//   ローカル系: +Y=長軸(四肢) / +X=前 / +Z=左
//   足は長軸が踵→爪先なので +X が長軸（longAxis:'x'）
function segInertia(spec, massKg, lenM){
  const kS=lenM*spec.rSag/100, kT=lenM*spec.rTra/100, kL=lenM*spec.rLon/100;
  if(spec.longAxis==='x') return [massKg*kL*kL, massKg*kS*kS, massKg*kT*kT];
  return [massKg*kS*kS, massKg*kL*kL, massKg*kT*kT];   // [Ixx,Iyy,Izz]
}
const rodI =(m,L,r)=>[m*(3*r*r+L*L)/12, m*r*r/2, m*(3*r*r+L*L)/12];   // 長軸=Y
const rodIx=(m,L,r)=>[m*r*r/2, m*(3*r*r+L*L)/12, m*(3*r*r+L*L)/12];   // 長軸=X
const boxI =(m,w,h,d)=>[m*(h*h+d*d)/12, m*(w*w+d*d)/12, m*(w*w+h*h)/12];

// ============================================================
//  骨格（体幹系。原点=股関節中点, +Y=上(体幹長軸), +X=前, +Z=左）
// ============================================================
function buildSkeleton(P){
  const S=P.athleteHeightM/DELEVA_M.ref.statureM, M=P.athleteMassKg;
  const mm=x=>x/1000*S;
  const L={trunk:mm(DELEVA_M.trunk.len), head:mm(DELEVA_M.head.len),
           ua:mm(DELEVA_M.upperArm.len), fa:mm(DELEVA_M.forearm.len),
           hand:mm(DELEVA_M.hand.len), thigh:mm(DELEVA_M.thigh.len),
           shank:mm(DELEVA_M.shank.len), foot:mm(DELEVA_M.foot.len)};
  const kg=s=>DELEVA_M[s].mass/100*M;
  return {L, kg, S, M,
    shoulderHalf:P.shoulderHalfM, hipHalf:P.hipHalfM,
    poleLen:P.poleLengthM, poleMass:P.poleMassKg,
    bootMass:P.bootMassKg, skiMass:P.skiMassKg+P.bindingMassKg, skiLen:P.skiLengthM};
}

// 時刻tの各剛体の {R(体幹系), p(CM・体幹系), m, I(ローカル主慣性)}
function poseAt(sk, ang, t, opt){
  opt=opt||{};
  const A=ang(t), B=[];
  const L=sk.L, kg=sk.kg;
  const dn=[0,-1,0];
  const push=(m,I,R,p)=>{ if(m>0) B.push({m:m,I:I,R:R,p:p}); };

  // --- 体幹 + 頭 ---
  const mTrunk=kg('trunk'), mHead=kg('head');
  if(opt.mergeTrunkHead){
    const pT=[0,(1-DELEVA_M.trunk.cm/100)*L.trunk,0];
    const pH=[0,L.trunk+(1-DELEVA_M.head.cm/100)*L.head,0];
    const m=mTrunk+mHead, c=vs(vadd(vs(pT,mTrunk),vs(pH,mHead)),1/m);
    const IT=segInertia(DELEVA_M.trunk,mTrunk,L.trunk), IH=segInertia(DELEVA_M.head,mHead,L.head);
    const dT=vsub(pT,c), dH=vsub(pH,c);
    const I=[IT[0]+mTrunk*(dT[1]*dT[1]+dT[2]*dT[2])+IH[0]+mHead*(dH[1]*dH[1]+dH[2]*dH[2]),
             IT[1]+mTrunk*(dT[0]*dT[0]+dT[2]*dT[2])+IH[1]+mHead*(dH[0]*dH[0]+dH[2]*dH[2]),
             IT[2]+mTrunk*(dT[0]*dT[0]+dT[1]*dT[1])+IH[2]+mHead*(dH[0]*dH[0]+dH[1]*dH[1])];
    push(m,I,mI(),c);
  } else {
    push(mTrunk, segInertia(DELEVA_M.trunk,mTrunk,L.trunk), mI(),
         [0,(1-DELEVA_M.trunk.cm/100)*L.trunk,0]);
    push(mHead,  segInertia(DELEVA_M.head, mHead, L.head),  mI(),
         [0,L.trunk+(1-DELEVA_M.head.cm/100)*L.head,0]);
  }

  // --- 腕 ---
  const arms = opt.singleArm ? [1] : [1,-1];
  for(let ai=0; ai<arms.length; ai++){
    const side=arms[ai];
    const spread = side>0 ? A.spreadL : A.spreadR;
    const sh=[0,L.trunk,side*sk.shoulderHalf];
    const Rs=eulerXYZ(side*spread,0,A.swing);
    const ua=mv(Rs,dn);
    push(kg('upperArm'), segInertia(DELEVA_M.upperArm,kg('upperArm'),L.ua), Rs,
         vadd(sh,vs(ua,DELEVA_M.upperArm.cm/100*L.ua)));
    const el=vadd(sh,vs(ua,L.ua));
    const Rf=mmul(Rs,rotZ(A.elbow));
    const fd=mv(Rf,dn);
    push(kg('forearm'), segInertia(DELEVA_M.forearm,kg('forearm'),L.fa), Rf,
         vadd(el,vs(fd,DELEVA_M.forearm.cm/100*L.fa)));
    const wr=vadd(el,vs(fd,L.fa));
    push(kg('hand'), segInertia(DELEVA_M.hand,kg('hand'),L.hand), Rf,
         vadd(wr,vs(fd,DELEVA_M.hand.cm/100*L.hand)));
    if(sk.poleMass>0){
      push(sk.poleMass, rodI(sk.poleMass,sk.poleLen,0.011), Rf,
           vadd(el,mv(Rf,[0.02,-0.72,0])));
    }
  }

  // --- 脚 + ブーツ + スキー ---
  if(!opt.armsOnly) for(let si=0; si<2; si++){
    const side = si===0 ? 1 : -1;
    const hp=[0,0,side*sk.hipHalf];
    const Rh=rotZ(A.hip), td=mv(Rh,dn);
    push(kg('thigh'), segInertia(DELEVA_M.thigh,kg('thigh'),L.thigh), Rh,
         vadd(hp,vs(td,DELEVA_M.thigh.cm/100*L.thigh)));
    const kn=vadd(hp,vs(td,L.thigh));
    const Rk=mmul(Rh,rotZ(-A.knee)), sd=mv(Rk,dn);
    push(kg('shank'), segInertia(DELEVA_M.shank,kg('shank'),L.shank), Rk,
         vadd(kn,vs(sd,DELEVA_M.shank.cm/100*L.shank)));
    const an=vadd(kn,vs(sd,L.shank));
    const Ra=mmul(Rk,rotZ(A.ankle));
    const heel=vadd(an,mv(Ra,[-0.06,-0.08,0]));
    push(kg('foot'), segInertia(DELEVA_M.foot,kg('foot'),L.foot), Ra,
         vadd(heel,mv(Ra,[DELEVA_M.foot.cm/100*L.foot,0,0])));
    if(sk.bootMass>0) push(sk.bootMass, boxI(sk.bootMass,0.28,0.17,0.14), Ra, vadd(an,mv(Ra,[0.03,-0.08,0])));
    if(sk.skiMass>0)  push(sk.skiMass,  rodIx(sk.skiMass,sk.skiLen,0.05),  Ra, vadd(an,mv(Ra,[0.14,-0.175,0])));
  }
  return B;
}

// I(q) と h_rel(q,q̇) を体幹系で組む（全身CMまわり）
function inertiaAndHrel(sk, ang, t, opt){
  const h=1e-4;
  const B0=poseAt(sk,ang,t,opt), Bp=poseAt(sk,ang,t+h,opt), Bm=poseAt(sk,ang,t-h,opt);
  const n=B0.length;
  let Mtot=0, c=[0,0,0], cd=[0,0,0];
  for(let i=0;i<n;i++){
    Mtot+=B0[i].m;
    c =vadd(c, vs(B0[i].p,B0[i].m));
    cd=vadd(cd,vs(vs(vsub(Bp[i].p,Bm[i].p),1/(2*h)),B0[i].m));
  }
  c=vs(c,1/Mtot); cd=vs(cd,1/Mtot);
  let I=[[0,0,0],[0,0,0],[0,0,0]], hrel=[0,0,0];
  for(let i=0;i<n;i++){
    const m=B0[i].m, Il=B0[i].I, R=B0[i].R, p=B0[i].p;
    const Iw=mmul(R,mmul(M3(Il[0],0,0,0,Il[1],0,0,0,Il[2]),mT(R)));
    const d=vsub(p,c), d2=vdot(d,d);
    const par=M3(d2-d[0]*d[0], -d[0]*d[1], -d[0]*d[2],
                 -d[1]*d[0], d2-d[1]*d[1], -d[1]*d[2],
                 -d[2]*d[0], -d[2]*d[1], d2-d[2]*d[2]);
    I=madd(I, madd(Iw, msc(par,m)));
    const Rd=msc(madd(Bp[i].R,msc(Bm[i].R,-1)),1/(2*h));
    const W=mmul(Rd,mT(R));
    const w=[(W[2][1]-W[1][2])/2,(W[0][2]-W[2][0])/2,(W[1][0]-W[0][1])/2];
    const pd=vs(vsub(Bp[i].p,Bm[i].p),1/(2*h));
    hrel=vadd(hrel, vadd(mv(Iw,w), vs(vcrs(d,vsub(pd,cd)),m)));
  }
  return {I:I,hrel:hrel,Mtot:Mtot,com:c,nBodies:n};
}

// H0固定で姿勢を積分（RK4）
function integrate(sk, ang, opt){
  const H0=opt.H0, T=opt.T, dt=opt.dt;
  const steps=Math.round(T/dt);
  let q=qnorm(opt.q0.slice());
  const out=[];
  const omegaBody=(t,qq)=>{
    const R=qToM(qq);
    const r=inertiaAndHrel(sk,ang,t,opt);
    return mv(minv(r.I), vsub(mv(mT(R),H0), r.hrel));
  };
  const every=opt.sample||1;
  for(let s=0;s<=steps;s++){
    const t=s*dt;
    if(s%every===0 || s===steps){
      const R=qToM(q);
      out.push({t:t, q:q.slice(), L:mv(R,[0,1,0]), wb:omegaBody(t,q)});
    }
    if(s===steps) break;
    const k1=qDot(q,omegaBody(t,q));
    const q2=qnorm(q.map((v,i)=>v+0.5*dt*k1[i]));
    const k2=qDot(q2,omegaBody(t+dt/2,q2));
    const q3=qnorm(q.map((v,i)=>v+0.5*dt*k2[i]));
    const k3=qDot(q3,omegaBody(t+dt/2,q3));
    const q4=qnorm(q.map((v,i)=>v+dt*k3[i]));
    const k4=qDot(q4,omegaBody(t+dt,q4));
    q=qnorm(q.map((v,i)=>v+dt/6*(k1[i]+2*k2[i]+2*k3[i]+k4[i])));
  }
  return out;
}

// tilt = asin(|L̂·Ĥ|) と、Hまわりの累積周回角 φ
function analyze(traj,H0){
  const Hh=vunit(H0);
  const proj=v=>vunit(vsub(v,vs(Hh,vdot(v,Hh))));
  let phi=0, prev=proj(traj[0].L);
  return traj.map((f,i)=>{
    if(i>0){ const cur=proj(f.L);
      phi+=Math.atan2(vdot(vcrs(prev,cur),Hh), vdot(prev,cur)); prev=cur; }
    return {t:f.t,
            tiltDeg:Math.asin(Math.min(1,Math.abs(vdot(vunit(f.L),Hh))))*180/Math.PI,
            phiDeg:phi*180/Math.PI, wb:f.wb};
  });
}

module.exports={MODEL_VERSION:MODEL_VERSION,DELEVA_M:DELEVA_M,buildSkeleton:buildSkeleton,poseAt:poseAt,
  inertiaAndHrel:inertiaAndHrel,integrate:integrate,analyze:analyze,
  vlen:vlen,vunit:vunit,vdot:vdot,vcrs:vcrs,qToM:qToM,mv:mv,mT:mT,minv:minv,
  segInertia:segInertia};
