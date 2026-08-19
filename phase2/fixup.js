'use strict';
const fs=require('fs');
let src=fs.readFileSync('table.js','utf8');
const PT=eval(src+';PHYSICS_TABLE');
for(const k of ['p0','p025']){
  const o=PT[k], N=o.tilt[0].length-1;
  o.somTotal=o.phi.map(a=>+(a[N]-a[0]).toFixed(1));
  o.twistTotal=o.psi.map(a=>+(a[N]-a[0]).toFixed(1));
  o.tiltPeak=o.tilt.map(a=>+Math.max.apply(null,a.map(Math.abs)).toFixed(2));
}
const j=o=>JSON.stringify(o);
let s='var PHYSICS_TABLE = {\n';
s+='  meta:{ airtimeS:1.65, tauSamples:'+(PT.p0.tilt[0].length)+', takeoffTiltsDeg:'+j(PT.meta.takeoffTiltsDeg)+',\n';
s+='         segmentTable:"de Leva (1996) Table 4, male",\n';
s+='         note:"Phase 2 物理モード。オフラインで解いた結果を (tilt,phi,psi) で保持。phi/psi は絶対値（phi は離地角24.41度から始まる）" },\n';
for(const k of ['p0','p025']){
  const o=PT[k];
  s+='  '+k+':{ H0:'+j(o.H0)+', somTotal:'+j(o.somTotal)+', twistTotal:'+j(o.twistTotal)+', tiltPeak:'+j(o.tiltPeak)+',\n';
  s+='    tilt:'+j(o.tilt)+',\n    phi:'+j(o.phi)+',\n    psi:'+j(o.psi)+' },\n';
}
s+='};\n';
fs.writeFileSync('table.js',s);
console.log('離地tilt :', PT.meta.takeoffTiltsDeg.join('  '));
for(const k of ['p0','p025']){
  console.log(`\n[${k==='p0'?'ポール無し':'ポール0.25kg'}]`);
  console.log('  宙返り総量:', PT[k].somTotal.join('  '));
  console.log('  ひねり総量:', PT[k].twistTotal.join('  '));
  console.log('  tiltピーク:', PT[k].tiltPeak.join('  '));
  // ひねり360°に必要な離地tiltを線形補間
  const T=PT.meta.takeoffTiltsDeg, W=PT[k].twistTotal;
  for(let i=0;i<W.length-1;i++) if(W[i]<360&&W[i+1]>=360)
    console.log('  → ひねり360°に必要な離地tilt ≈ '+ (T[i]+(360-W[i])/(W[i+1]-W[i])*(T[i+1]-T[i])).toFixed(1)+'°');
}
