/** 鲁棒性扫描：不同 dt / 圆轨迹幅度下检查数值稳定（不出现 NaN）。 */
import { readParams } from './params.js';
import { Engine } from './engine.js';

const { params: base } = await readParams();

function trial(label, params, setup, seconds) {
  const eng = new Engine(params);
  eng.run(1.0); // 稳定
  setup(eng);
  let bad = 0;
  let maxQ = 0;
  for (let k = 0; k < seconds * 20; k++) {
    eng.run(0.05);
    const s = eng.sim;
    for (let i = 0; i < 6; i++) if (!Number.isFinite(s.x[i])) bad++;
    for (let i = 0; i < 3; i++) maxQ = Math.max(maxQ, Math.abs(s.x[i]));
  }
  console.log(`${label.padEnd(34)} bad=${String(bad).padEnd(6)} maxT=${Math.max(...eng.sim.tension).toFixed(1).padStart(8)}  x=${eng.sim.x.slice(0, 3).map((v) => v.toFixed(3)).join(',')}  ang=${eng.sim.x.slice(3).map((v) => v.toFixed(2)).join(',')}`);
  return bad === 0;
}

let ok = true;
for (const phys of [1000, 2000, 4000]) {
  const p = JSON.parse(JSON.stringify(base));
  p.loop.physics_rate_hz = phys;
  p.loop.controller_rate_hz = Math.min(1000, phys);
  p.loop.driver_rate_hz = Math.min(1000, phys);
  ok = trial(`dt=1/${phys} 圆轨迹`, p, (e) => e.startCircle(), 8) && ok;
}
for (const amp of [0.04, 0.08, 0.12]) {
  const p = JSON.parse(JSON.stringify(base));
  p.trajectory.xy_amplitude = amp;
  p.trajectory.roll_amplitude = amp * 0.8;
  p.trajectory.pitch_amplitude = amp * 1.2;
  p.trajectory.yaw_amplitude = amp * 4;
  ok = trial(`圆轨迹幅度 ${amp}`, p, (e) => e.startCircle(), 10) && ok;
}
ok = trial('goto left -> right -> home', base, (e) => { e.gotoNamed('left', 1.5); }, 4) && ok;
console.log(ok ? 'ALL STABLE' : 'UNSTABLE CASES FOUND');
process.exit(ok ? 0 : 1);
