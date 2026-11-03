/** 码垛单段复现：直接执行「原点 -> (-0.2,0.42,-0.22)」段，与直线运动对比。 */
import { readParams } from './params.js';
import { Engine } from './engine.js';

const { params } = await readParams();
const eng = new Engine(params);
eng.run(3);
// 复刻 palletize 的第一段，但段时间为 4 s（更慢）
const qT = Float64Array.from([-0.2, 0.42, -0.22, 0, 0, 0]);
eng.traj.goto(qT, 4.0);
for (let k = 0; k < 60; k++) {
  eng.run(0.1);
  const x = eng.sim.x, q = eng.qTarget;
  const e = Math.hypot(x[0] - q[0], x[1] - q[1], x[2] - q[2]);
  console.log(`t=${eng.t.toFixed(1)} qT=${q.slice(0, 3).map((v) => v.toFixed(3)).join(',')} q=${x.slice(0, 3).map((v) => v.toFixed(3)).join(',')} e=${(e * 1000).toFixed(0)}mm Tmin=${Math.min(...eng.sim.tension).toFixed(1)} Tmax=${Math.max(...eng.sim.tension).toFixed(1)}`);
}
