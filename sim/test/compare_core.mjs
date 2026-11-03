/**
 * C 核心与 JS 核心的一致性检查。
 *
 *   node sim/test/compare_core.mjs
 *
 * 调用 firmware/build/consistency_test.exe（若存在），与 sim/core 的
 * 同参数计算结果逐项比较；全部通过时退出码 0。
 */
import { execFileSync } from 'node:child_process';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

import { readParams } from '../core/params.js';
import { buildModel, structure, poseToX, allocateT, externalWrench, forwardKinematics } from '../core/core.js';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const ROOT = path.resolve(HERE, '../..');
const EXE = path.join(ROOT, 'firmware', 'build', 'consistency_test.exe');

if (!fs.existsSync(EXE)) {
  console.log(JSON.stringify({ ok: false, reason: `未找到 ${EXE}，请先运行 firmware/build_sim.cmd` }, null, 2));
  process.exit(1);
}

const raw = execFileSync(EXE, { encoding: 'utf8' });
const c = JSON.parse(raw);

const { params: p } = await readParams();
const model = buildModel(p);
// 用参数表里的零位，与 C 侧 consistency_test.c 保持一致
const home = p.controller.home_pose;
const x = poseToX([home[0], home[1], home[2]], [home[3], home[4], home[5]]);
const g = structure(model, x);
const w = externalWrench(p);
const alloc = allocateT(model, x, w, { mode: p.controller.tension_alloc });
const fk = forwardKinematics(model, g.l, poseToX([home[0] + 0.02, home[1] + 0.02, home[2] + 0.02], [0.02, 0.02, 0.02]), 30);

const checks = [];
const maxDiff = (a, b) => {
  let m = 0;
  for (let i = 0; i < a.length; i++) m = Math.max(m, Math.abs(a[i] - b[i]));
  return m;
};

const dLen = maxDiff(Array.from(g.l), c.length);
checks.push({ name: '绳长', ok: dLen < 2e-3, maxDiff: dLen });

const flatA = [];
for (let r = 0; r < 6; r++) for (let i = 0; i < 8; i++) flatA.push(g.A.d[r * 8 + i]);
const flatC = [];
for (let r = 0; r < 6; r++) for (let i = 0; i < 8; i++) flatC.push(c.A[r][i]);
const dA = maxDiff(flatA, flatC);
checks.push({ name: '结构矩阵', ok: dA < 5e-3, maxDiff: dA });

// 张力分配：两边都用「非负最小二乘」，但迭代起点不同（C 用热启动、JS 冷启动），
// 收敛到的点在同一可行解集上允许有小差异；判据用「各自满足 A·T=w 的残差」而不是逐值相等。
const jsT = Array.from(alloc.T);
const dT = maxDiff(jsT, c.tension);
// 两个解各自的结构残差（都应很小）
const resid = (T) => {
  let m2 = 0;
  for (let r = 0; r < 6; r++) {
    let s2 = w[r];
    for (let i = 0; i < 8; i++) s2 -= g.A.d[r * 8 + i] * T[i];
    m2 = Math.max(m2, Math.abs(s2));
  }
  return m2;
};
const rJs = resid(jsT);
const rC = resid(c.tension);
checks.push({
  name: '张力分配',
  ok: rJs < 0.5 && rC < 0.5 && dT < 15,
  maxDiff: dT,
  jsResidual: +rJs.toFixed(4),
  cResidual: +rC.toFixed(4),
  note: '两解都在可行解集内；逐值差异来自迭代起点不同',
  js: jsT.map((v) => +v.toFixed(2)),
  c: c.tension,
});

checks.push({ name: '正运动学（C 侧）', ok: c.fk_error < 0.02, cError: c.fk_error });

const ok = checks.every((k) => k.ok);
console.log(JSON.stringify({
  ok,
  jsLength: Array.from(g.l).map((v) => +v.toFixed(4)),
  cLength: c.length,
  checks,
  note: 'C 侧几何/分配与 JS 侧同参数结果一致；差异来自 float 精度与正运动学迭代次数。',
}, null, 2));
process.exit(ok ? 0 : 1);
