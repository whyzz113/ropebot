/** 冒烟测试：悬停 -> 位姿 -> 圆轨迹，输出时序摘要并校验数值稳定。 */
import { readParams } from './params.js';
import { Engine } from './engine.js';

const { params, errors } = await readParams();
if (errors.length) console.log('参数错误:', errors);
const eng = new Engine(params);
const show = (label, s) => {
  console.log(`--- ${label} ---`);
  console.log('q     ', s.q.map((v) => v.toFixed(3)).join(' '));
  console.log('qT    ', s.qTarget.map((v) => v.toFixed(3)).join(' '));
  console.log('T     ', s.T.join(' '));
  console.log('Tref  ', s.Tref.join(' '));
  console.log('cur   ', s.acts.map((a) => a.cur.toFixed(2)).join(' '));
  console.log('mvel  ', s.acts.map((a) => a.motorVel.toFixed(0)).join(' '));
  console.log('contact', s.contact, 'feasible', s.feasible);
};

const t0 = performance.now();
show('悬停 3 s', eng.run(3));
eng.gotoNamed('left', 2.0);
show('goto left 4 s', eng.run(4));
eng.gotoNamed('home', 2.0);
eng.run(3);
eng.startCircle();
for (let k = 1; k <= 6; k++) show(`圆轨迹 ${k} s`, eng.run(1));
let bad = 0;
for (const h of eng.history) for (const v of h.x) if (!Number.isFinite(v) || Math.abs(v) > 2) bad++;
console.log('异常样本:', bad, '样本数', eng.history.length);
const t1 = performance.now();
console.log(`性能：${((t1 - t0) / 1000).toFixed(2)} s 模拟 ${eng.t.toFixed(1)} s，速度 ${((eng.t * 1000) / (t1 - t0)).toFixed(1)}x 实时`);
process.exit(bad === 0 ? 0 : 1);
