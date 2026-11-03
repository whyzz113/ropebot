/**
 * 控制律（与固件 firmware/core/controller.c 一一对应）。
 *
 *   轨迹目标位姿 q_ref
 *     ├─> 张力分配 T_ref = allocateT(w)，w = 重力(+负载) − 外部接触力
 *     ├─> 绳长规划 l_ref = l(q_ref) − T_ref / k_c            （弹性预补偿，约 1~4 mm）
 *     └─> 张力闭环 Δl_i = −K_T (T_ref_i − T_i)               （叠加在位置环上）
 *   输出与固件相同：每轴 { mode, cablePosTarget, cableVelTarget, motorTrqTarget }，
 *   驱动器为位置模式（HBS860H STEP/DIR），力矩前馈可选。
 */
import { structure, allocateT, externalWrench, cableStiffness, PI } from './core.js';
import { Pid6 } from './pid.js';

export const CMD_POS = 0;
export const CMD_VEL = 1;
export const CMD_TRQ = 2;
export const CMD_TRQ_VEL = 3;

/** 平板惯量（与 model.js 一致），用于加速度前馈 */
export function plateInertia(p) {
  const m = p.platform.mass;
  const a = 2 * p.platform.half_size_x;
  const b = 2 * p.platform.half_size_z;
  const c = Math.max(p.platform.thickness, 0.005);
  return [m * (b * b + c * c) / 12, m * (a * a + c * c) / 12, m * (a * a + b * b) / 12];
}

export function emptyCommands(n = 8) {
  return Array.from({ length: n }, () => ({ mode: CMD_POS, pos: 0, vel: 0, trq: 0 }));
}

/**
 * 带加速度限制的速率限制器：返回不超过 vMax、变化率不超过 aMax 的新值。
 * 一维二阶（位置-速度）限幅，等价于「梯形速度插值」。
 */
function rateLimit(desired, prev, velArr, i, dt, vMax, aMax) {
  const err = desired - prev;
  let v = velArr[i];
  if (Math.sign(err) !== Math.sign(v)) v = 0; // 反向时先减速到 0
  const vDes = Math.sign(err) * Math.min(vMax, Math.sqrt(2 * aMax * Math.abs(err)));
  const dv = Math.max(-aMax * dt, Math.min(aMax * dt, vDes - v));
  v += dv;
  let next = prev + v * dt;
  // 不允许越过目标
  if ((desired - prev) * (desired - next) < 0) {
    next = desired;
    v = 0;
  }
  velArr[i] = v;
  return next;
}

export class Controller {
  constructor(params, model) {
    this.p = params;
    this.model = model;
    this.n = model.nCables;
    this.cmd = emptyCommands(this.n);
    this.pid = new Pid6(params.controller.pid, params);
    this.reset();
  }

  reset() {
    const p = this.p;
    const hp = p.controller.home_pose;
    const x0 = Float64Array.from(hp);
    this.l0 = Float64Array.from(structure(this.model, x0).l);
    this.tRef = new Float64Array(this.n);
    this.tFilt = new Float64Array(this.n);
    this.lRef = new Float64Array(this.n);
    this.loadOffset = 0;
    this.contactForceY = 0;
    this.prevQ = null;
    this.prevV = null;
    this.ffAcc = new Float64Array(6);
    this.inertia = plateInertia(p);
    const r = allocateT(this.model, x0, externalWrench(p), { mode: p.controller.tension_alloc });
    this.tRef.set(r.T);
    this.tFilt.set(r.T);
    for (let i = 0; i < this.n; i++) {
      const k = cableStiffness(p, this.l0[i]);
      this.lRef[i] = this.l0[i] - this.tRef[i] / k;
    }
    this.feasible = r.feasible;
    this.diag = { T: r.T, fmin: r.fmin, fmax: r.fmax, residual: r.residual, fcMargin: r.fcMargin };
    // 绳长参考限速器状态
    this.lRefPrev = Float64Array.from(this.lRef);
    this.lRefVel = new Float64Array(this.n);
    this.vFilt = new Float64Array(6);
    this.aFilt = new Float64Array(6);
    if (this.pid) this.pid.reset();
  }

  setLoad(kg) {
    this.loadOffset = Math.max(0, kg);
  }

  /** 一个控制周期
   * @param {number} dt 控制周期
   * @param {ArrayLike<number>} qTarget 目标位姿
   * @param {ArrayLike<number>} measT 实测张力
   * @param {object} meas 其他测量量：{ contact, q }（q 为实测位姿，任务空间 PID 用）
   */
  step(dt, qTarget, measT, meas = {}) {
    const p = this.p;
    const c = p.controller;
    const st = structure(this.model, qTarget);
    // 外部接触力（平台压在地面上时地面支撑抵消重力）
    this.contactForceY = meas.contact ? -(p.platform.mass + this.loadOffset) * p.platform.gravity : 0;
    const w = externalWrench(p, [0, this.contactForceY, 0]);
    w[1] += this.loadOffset * p.platform.gravity;
    // 加速度前馈：平台做姿态/平动加减速时，绳力需额外提供 m*a（牛顿-欧拉中的
    // 惯性项）。用带通滤波取轨迹目标位姿的二阶差分（滤掉逐周期数值噪声）。
    if (this.prevQ && this.vFilt) {
      const dtl = Math.max(dt, 1e-6);
      const wc = 2 * Math.PI * (c.ff_accel_cutoff_hz ?? 5);
      const k = dtl / (dtl + 1 / wc);
      const v = new Float64Array(6);
      for (let i = 0; i < 6; i++) v[i] = this.vFilt[i] + k * ((qTarget[i] - this.prevQ[i]) / dtl - this.vFilt[i]);
      const acc = new Float64Array(6);
      for (let i = 0; i < 6; i++) acc[i] = this.aFilt[i] + k * ((v[i] - this.vFilt[i]) / dtl - this.aFilt[i]);
      this.vFilt.set(v);
      this.aFilt.set(acc);
      const m = p.platform.mass + this.loadOffset;
      const filtered = [
        m * acc[0], m * acc[1], m * acc[2],
        this.inertia[0] * acc[3], this.inertia[1] * acc[4], this.inertia[2] * acc[5],
      ];
      for (let i = 0; i < 6; i++) {
        this.ffAcc[i] = filtered[i];
        w[i] += filtered[i];
      }
    } else {
      this.prevQ = Float64Array.from(qTarget);
      this.prevV = new Float64Array(6);
      this.vFilt = new Float64Array(6);
      this.aFilt = new Float64Array(6);
      this.ffAcc = new Float64Array(6);
      this.inertia = plateInertia(p);
    }
    this.prevQ.set(qTarget);

    // ---- 任务空间 PID（可选） ----------------------------------------
    // 用实测位姿 qMeas 与目标位姿求 6 维误差，PID 输出「额外的力/力矩」，
    // 与重力前馈、加速度前馈一起构成 w = A T 的右侧。
    // 注意：PID 只负责「误差修正」，重力由前馈承担，这样积分项不会长期
    // 扛着 39 N 的重力，抗饱和与超调都好得多。
    if (this.pid && meas.q) {
      const pidOut = this.pid.step(dt, qTarget, meas.q);
      for (let i = 0; i < 6; i++) w[i] += pidOut[i];
    }

    const r = allocateT(this.model, qTarget, w, { mode: c.tension_alloc });
    const a = 1 - Math.exp(-2 * PI * 10 * dt);
    for (let i = 0; i < this.n; i++) this.tRef[i] += (r.T[i] - this.tRef[i]) * a;
    this.feasible = r.feasible;
    this.diag.T = r.T;
    this.diag.fmin = r.fmin;
    this.diag.fmax = r.fmax;
    this.diag.residual = r.residual;
    this.diag.fcMargin = r.fcMargin;

    // 期望绳长：弹性预补偿用「实测张力」而不是「期望张力」。
    //
    // 原设计用的是期望张力 T_ref：l_ref = l(q_ref) - T_ref/k。钢丝刚度 k ≈ 2.5 kN/m，
    // 期望张力沿绳分布不均（最小范数解 4~13 N），于是各绳预补偿量差 1~3 mm，
    // 等于给平台加了一个「隐藏的位移偏置」；张力环增益 K_T = 2e-5 m/N 只能抵消这个
    // 偏置的很小一部分，平台因此停在偏差几厘米的位置。改用实测张力后，
    // 期望绳长与「目标位姿 + 当前张力」自洽，张力环只需修正小偏差。
    // 注意 kT 不能太大：张力环外环与位置环并联，过大相当于在电机回路上加弹簧，
    // 会引发低频振荡（实测 kT = 4e-4 m/N 时圆轨迹发散）。
    // 绳长指令限速：按目标位姿中「平移速度」的上限换算成卷筒收放绳速度，
    // 保证张力分配算法带来的参考跳变不会超过执行器能力。
    //   vmax_cable = max_trans_vel * 1.5
    // 注意：卷筒加速度限制不必加（位置环增益 Kp=30 1/s 本身就限定了速度），
    // 加了反而会让张力预补偿项跟不上，形成数厘米的稳态偏差。
    const vMax = c.lref_vmax ?? (c.max_trans_vel ?? 0.45) * 1.5;
    const aMax = c.lref_amax ?? 100.0; // 实际上不限制加速度
    if (!this.lRefPrev) {
      this.lRefPrev = Float64Array.from(this.lRef);
      this.lRefVel = new Float64Array(this.n);
    }
    // 弹性预补偿 + 张力闭环：
    //   l_ref = l(q_ref) - T_ref/k_current - kT (T_ref - T_meas)
    // 第一项把「期望张力所需的伸长量」收回到卷筒里（k_current 为当前绳长下的
    // 实测刚度 EA/l，随绳长变化，保证收绳量与实际弹性伸长自洽）；
    // 第二项由张力偏差做精修。两项共同决定平台的实际位置误差，
    // 因此 kT 不宜过大（>1e-4 m/N 时外环与位置环耦合会引起低频振荡）。
    const af = 1 - Math.exp(-2 * PI * 25 * dt);
    const kT = c.tension_kp;
    for (let i = 0; i < this.n; i++) {
      this.tFilt[i] += (measT[i] - this.tFilt[i]) * af;
      const k = cableStiffness(p, st.l[i]);
      // 张力环（比例）：dl = -kT (T_ref - T_meas)，限幅 +/-2 mm。
      const dT = this.tRef[i] - this.tFilt[i];
      const dL = Math.max(-2e-3, Math.min(2e-3, -kT * dT));
      const desired = st.l[i] - this.tRef[i] / k + dL;
      // 结果指令向量 = 限速后的参考，避免整机分辨率 3.9 µm/步 造成的大幅跳变；
      // 限速对 1 kHz 刷新足够（vMax=0.3 m/s 即每周期 0.3 mm），不影响跟踪。
      this.lRef[i] = rateLimit(desired, this.lRefPrev[i], this.lRefVel, i, dt, vMax, aMax);
      this.lRefPrev[i] = this.lRef[i];
      const cmd = this.cmd[i];
      cmd.mode = CMD_POS;
      cmd.pos = this.lRef[i];
      cmd.vel = 0;
      // 力矩前馈（电机轴，收绳方向为负）
      cmd.trq = -(this.tRef[i] * p.winders.drum_radius) / p.actuators.gear_ratio;
    }
    return this.cmd;
  }
}
