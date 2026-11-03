/**
 * 力学自检：`node sim/core/check_mechanics.mjs`
 * 输出 JSON；全部 ok 时退出码 0。构建固件（build_firmware）前也跑同一判据。
 */
import { readParams } from './params.js';
import { buildModel, structure, poseToX, allocateT, externalWrench, forwardKinematics } from './core.js';

const { params: p, errors } = await readParams();
const model = buildModel(p);
const out = { errors, checks: [] };

const w0 = externalWrench(p);
const allPullPossible = w0[1] > 0;

// 严格 8 角拓扑（topology: 'corner'）下，A 的 My 行恒为 0（偏航不可控），
// 且零空间缺少全正向量。这些是几何事实，不是参数错误 —— 在输出里标注清楚，
// 免得把「模型本身的退化」误读成「参数配错了」。
const cornerTopo = p.winders?.topology === 'corner';
if (cornerTopo) {
  out.topologyNote = '严格 8 角拓扑：8 电机在框架 8 角 + 8 绳接平台 8 角。'
    + '该构型下 A 的 My 行恒为 0（绕竖直轴的力矩不可控），'
    + '且上下锚点共棱导致部分绳无法绷紧。tilt_y / corner 位姿必然失败。'
    + '若需要 6 自由度全可控，请改用 topology: "edge"。';
}

for (const pose of p.poses) {
  const x = poseToX(pose.p, pose.rpy);
  const s = structure(model, x);
  const r = allocateT(model, x, w0, { mode: p.controller.tension_alloc });
  // 与 allocateT 的判据保持一致：每根绳都要真正受拉（> t_min 的一个比例），
  // 且残差相对量级可忽略。
  const ok = allPullPossible && r.feasible;
  out.checks.push({
    name: `pose:${pose.name}`,
    ok,
    l: Array.from(s.l).map((v) => +v.toFixed(4)),
    T: Array.from(r.T).map((v) => +v.toFixed(1)),
    belowTmin: Array.from(r.T).filter((v) => v < p.cables.t_min).length,
    residual: +r.residual.toFixed(6),
    fcMargin: +r.fcMargin.toFixed(2),
    hint: ok ? undefined
      : (cornerTopo
        ? '严格 8 角拓扑的几何退化：该姿态需要的力矩在 A 的零空间里（无法由绳张力产生）'
        : '锚点不在平台上方的姿态无法靠 8 根绳同时受拉维持'),
  });
}

// 正运动学：绳长 -> 位姿 应可还原（用 home 位姿附近的小扰动，保证迭代收敛）
const x0 = poseToX([0, 0.62, 0], [0, 0, 0]);
const s0 = structure(model, x0);
const xr = forwardKinematics(model, s0.l, poseToX([0.01, 0.60, 0.01], [0.01, 0, 0.01]), 200);
let e = 0;
for (let k = 0; k < 6; k++) e = Math.max(e, Math.abs(xr[k] - x0[k]));
// 80 次迭代后残差约 3e-5 m（绳长 0.6 m 量级，相对误差 5e-5），足够工程使用
out.checks.push({ name: 'forward-kinematics', ok: e < 1e-4, maxErr: e });

// 结构矩阵秩：对「物理可实现」的外力（竖直分量为正 = 平台被向下压/自重），
// 非负张力应能在几 % 误差内平衡。纯拉力系统对任意力矩组合只能近似平衡
// （8 绳对 6 自由度是冗余的，但非负约束会限制可达力旋量锥），
// 因此这里用相对误差判据：残差 / 总外力 < 6%。
let rr = 0;
let rrRel = 0;
for (let t = 0; t < 5; t++) {
  const w = new Float64Array(6);
  w[1] = 20 + Math.random() * 30;           // 竖直向下
  w[0] = (Math.random() - 0.5) * 10;
  w[2] = (Math.random() - 0.5) * 10;
  w[3] = (Math.random() - 0.5) * 3;
  w[4] = (Math.random() - 0.5) * 3;
  w[5] = (Math.random() - 0.5) * 3;
  const r = allocateT(model, x0, w, { mode: 2 });
  rr = Math.max(rr, r.residual);
  rrRel = Math.max(rrRel, r.residual / Math.abs(w[1]));
}
out.checks.push({ name: 'structure-rank-8x6', ok: rrRel < 0.06, residual: rr, relative: rrRel });

// w = A T 一致性
const r0 = allocateT(model, x0, w0, { mode: 1 });
const A = structure(model, x0).A;
const rec = new Float64Array(6);
for (let r2 = 0; r2 < 6; r2++) for (let i = 0; i < 8; i++) rec[r2] += A.d[r2 * 8 + i] * r0.T[i];
out.checks.push({
  name: 'w = A T 一致性',
  ok: Math.abs(rec[1] - w0[1]) < 1e-6 && Math.abs(rec[0]) < 1e-6,
  rec: Array.from(rec).map((v) => +v.toFixed(3)),
});

const G = p.actuators.gear_ratio, eff = p.actuators.gear_efficiency;
out.summary = {
  drum_torque_Nm: +(p.actuators.hold_torque * G * eff).toFixed(2),
  cable_tension_at_hold_N: +((p.actuators.hold_torque * G * eff) / p.winders.drum_radius).toFixed(0),
  cable_mm_per_motor_rev: +((2 * Math.PI / G) * p.winders.drum_radius * 1000).toFixed(3),
  resolution_um_per_step: +(((2 * Math.PI / ((360 / p.actuators.step_angle_deg) * p.actuators.driver_microstep)) / G) * p.winders.drum_radius * 1e6).toFixed(2),
  encoder_um: +((2 * Math.PI / p.actuators.encoder_counts_per_rev) / G * p.winders.drum_radius * 1e6).toFixed(2),
  motor_rpm_at_0p45_mps: +((0.45 / p.winders.drum_radius) * G * 60 / (2 * Math.PI)).toFixed(0),
};

console.log(JSON.stringify(out, null, 2));
process.exit(out.checks.every((c) => c.ok) && errors.length === 0 ? 0 : 1);
