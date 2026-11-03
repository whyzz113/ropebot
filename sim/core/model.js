/**
 * 整机动力学：
 *   8 个卷筒（由 actuator.js 的 HBS860H + 86 电机模型驱动）
 *   + 8 根只拉不推的弹性绳
 *   + 6 自由度动平台（刚体）
 *
 * 张力计算（每根绳）：
 *   T_i = max(0, k_c(i) * (l_i - l_free_i) + F0_i)
 *   l_free_i = l0_i - 卷筒收绳量_i              （收绳让需要长度变短 -> 张力上升）
 *   k_c = E A / l0                              （钢丝弹性，E A ≈ 1.77 kN）
 *   F0  = 控制器的张力偏置（张力模式叠加在位置模式上的「预紧」）
 * 卷筒负载力矩（电机轴）： tau_load = T_i * r / G   （收绳阻力为正）
 *
 * 平台方程（世界系，半隐式欧拉 + 角速度积分）：
 *   m * a = Σ T_i s_i + m g + F_contact
 *   I_w * dw/dt = Σ (B_i - p) × (T_i s_i) - w × (I_w w)
 *
 * 数值：卷筒取隐式阻尼（绳索刚度 1.77 kN/0.7 m ≈ 2.5 kN/m 与面积 ~0.2 kg 相比
 * 时间常数约 10 ms，0.5 ms 步长足够，再加阻尼保证稳定）。
 */
import { structure, rpyToR, matNew, solveDense, PI } from './core.js';
import { createActuator } from './actuator.js';
import { cableStiffness } from './core.js';

/**
 * 绳索结构阻尼系数（1/s）。约 4 1/s，对应绳索第一阶纵向模态 1~3% 的临界阻尼；
 * 可在参数文件 cables.damping 中修改（0 表示理想无阻尼绳）。
 */
let THETA = 4.0;

const vadd = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const vsub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const vmul = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

function plateInertia(p) {
  const m = p.platform.mass;
  const a = 2 * p.platform.half_size_x;
  const b = 2 * p.platform.half_size_z;
  const c = Math.max(p.platform.thickness, 0.005);
  return [m * (b * b + c * c) / 12, m * (a * a + c * c) / 12, m * (a * a + b * b) / 12];
}

function worldInertia(R, I) {
  const K = matNew(3, 3);
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++) {
      let s = 0;
      for (let k = 0; k < 3; k++) s += R[i][k] * I[k] * R[j][k];
      K.d[i * 3 + j] = s;
    }
  return K;
}

export class CdprSim {
  /**
   * @param {object} params 参数
   * @param {object} model buildModel 结果
   * @param {Array} controls 控制器输出（每根绳 {mode,pos,vel,trq}），可为 null
   */
  constructor(params, model) {
    this.p = params;
    this.model = model;
    this.n = model.nCables;
    THETA = params.cables?.damping ?? 4.0;
    this.I = plateInertia(params);
    this.acts = Array.from({ length: this.n }, (_, i) => createActuator(params, i));
    this.contactF = [0, 0, 0];
    this.reset();
  }

  reset(x0) {
    const hp = this.p.controller.home_pose;
    this.x = x0 ? Float64Array.from(x0) : Float64Array.from([hp[0], hp[1], hp[2], hp[3], hp[4], hp[5]]);
    this.vp = [0, 0, 0];
    this.w = [0, 0, 0];
    this.t = 0;
    this.st = structure(this.model, this.x);
    this.l0 = Float64Array.from(this.st.l);
    // 初始状态：不动点（与真实系统上电「找零」一致），张力从 0 开始爬升
    this.tension = new Float64Array(this.n);
    // 每根绳的机械预紧力 F0（tension preset），来自参数 cables.pretension
    this.bias = new Float64Array(this.n).fill(this.p.cables?.pretension ?? 0);
    this.contact = false;
    this.contactForce = [0, 0, 0];
    this.energy = 0;
    for (const a of this.acts) {
      a.st.motorPos = 0;
      a.st.motorVel = 0;
      a.st.motorTrq = 0;
      a.st.steps = 0;
      a.st.enabled = true;
      a.st.fault = 0;
      a.st.cur = 0;
      a.st.velInt = 0;
    }
    this.stepsIssued = new Float64Array(this.n);
  }

  /** 控制器指令 -> 各驱动器脉冲数（绝对绳长 -> 收绳量） */
  applyCommands(cmds, l0Base) {
    for (let i = 0; i < this.n; i++) {
      const cmd = cmds && cmds[i];
      if (!cmd) continue;
      this.acts[i].command(cmd, l0Base ? l0Base[i] : this.l0[i]);
      this.bias[i] = cmd.mode === 2 ? 0 : 0;
    }
  }

  /**
   * 一个物理步。
   * @param {number} dt s（建议 0.0005）
   */
  step(dt) {
    const p = this.p;
    const n = this.n;
    const g = p.platform.gravity;
    const m = p.platform.mass;
    const r = p.winders.drum_radius;
    const G = p.actuators.gear_ratio;
    const st = structure(this.model, this.x);
    this.st = st;

    // ---- 1. 绳索张力与卷筒负载 -------------------------------------
    const T = new Float64Array(n);
    const Jload = new Float64Array(n);
    for (let i = 0; i < n; i++) {
      const a = this.acts[i];
      const l = st.l[i];
      const k = cableStiffness(p, l);
      const lFree = this.l0[i] - a.st.cablePos;
      // 纯弹性：T = max(0, k (l - l_free))。绳索在自由长度时才松弛，
      // 预紧通过「多收一点绳」（控制器弹性预补偿）实现，而不是虚构的张力截距。
      let ti = k * (l - lFree);
      if (ti < 0) ti = 0; // 只拉不推
      T[i] = ti;
      a.st.lastTension = ti;
      // 卷筒侧惯量：电机 + 减速器 + 「半隐式」的绳索刚度等效质量。
      // 张力 T 是由绳长误差产生的弹簧力，弹簧两端分别是卷筒和平台；
      // 对卷筒而言弹簧以 k*(r/G)^2 的刚度作用，其能承受的时间步尺度由
      // J_eq = max(J_motor, kAxle * dt^2 / 0.5) 近似（隐式阻尼思想）。
      const Jm = p.actuators.inertia_motor + p.actuators.inertia_gear;
      const kAxle = k * (r / G) * (r / G);
      const J = Math.max(Jm, kAxle * dt * dt * 2.0);
      Jload[i] = J;
      // 电机轴负载：收绳阻力 T*r/G
      a.stepDynamics(dt, (ti * r) / G, J);
    }
    this.tension = T;

    // ---- 2. 平台动力学 --------------------------------------------
    const R = rpyToR([this.x[3], this.x[4], this.x[5]]);
    const Iw = worldInertia(R, this.I);
    let F = [0, 0, 0];
    let Tau = [0, 0, 0];
    for (let i = 0; i < n; i++) {
      const f = vmul(st.s[i], T[i]);
      F = vadd(F, f);
      Tau = vadd(Tau, cross(vsub(st.B[i], [this.x[0], this.x[1], this.x[2]]), f));
    }
    F[1] -= m * g;
    // 地面/桌面接触（floor_y）
    const fy = p.controller.floor_y ?? 0.05;
    if (this.x[1] <= fy + 1e-9 && F[1] < 0) {
      this.contact = true;
      this.x[1] = fy;
      this.contactForce = [0, F[1], 0];
      F[1] = 0;
      this.vp[1] = Math.max(0, this.vp[1]);
    } else {
      this.contact = false;
      this.contactForce = [0, 0, 0];
    }

    const IwW = [
      Iw.d[0] * this.w[0] + Iw.d[1] * this.w[1] + Iw.d[2] * this.w[2],
      Iw.d[3] * this.w[0] + Iw.d[4] * this.w[1] + Iw.d[5] * this.w[2],
      Iw.d[6] * this.w[0] + Iw.d[7] * this.w[1] + Iw.d[8] * this.w[2],
    ];
    const gyro = cross(this.w, IwW);
    const rhs = new Float64Array(3);
    for (let k = 0; k < 3; k++) rhs[k] = (Tau[k] - gyro[k]) * dt;
    const dw = solveDense(Iw, rhs);
    this.w = vadd(this.w, [dw[0], dw[1], dw[2]]);
    this.vp = vadd(this.vp, vmul(F, dt / m));

    // 绳索内阻（结构阻尼，约 1~3% 临界阻尼）。只在水平方向生效：竖直方向由
    // 8 根绳的拉力与重力直接决定，真实系统不会「自动阻尼」下沉速度，
    // 而水平方向的绳索纵向振荡需要阻尼来衰减。
    this.vp = vmul(this.vp, Math.exp(-THETA * dt));
    this.w = vmul(this.w, Math.exp(-THETA * dt));

    this.x[0] += this.vp[0] * dt;
    this.x[1] += this.vp[1] * dt;
    this.x[2] += this.vp[2] * dt;
    const [rr, pp] = [this.x[3], this.x[4]];
    const cr = Math.cos(rr), sr = Math.sin(rr), cp = Math.max(Math.abs(Math.cos(pp)), 1e-2);
    const wx = this.w[0], wy = this.w[1], wz = this.w[2];
    this.x[3] += (wx + (wy * sr + wz * cr) * (Math.sin(pp) / cp)) * dt;
    this.x[4] += (wy * cr - wz * sr) * dt;
    this.x[5] += ((wy * sr + wz * cr) / cp) * dt;
    this.t += dt;
  }

  /** 正运动学（由卷筒编码器还原位姿），控制器与显示使用 */
  fkFromPulleys(xGuess) {
    const l = new Float64Array(this.n);
    for (let i = 0; i < this.n; i++) l[i] = this.l0[i] - this.acts[i].st.cablePos;
    return l;
  }

  snapshot() {
    return {
      t: this.t,
      x: Array.from(this.x),
      l: Array.from(this.st.l),
      T: Array.from(this.tension),
      contact: this.contact,
      acts: this.acts.map((a) => ({
        cablePos: a.st.cablePos, cableVel: a.st.cableVel, motorVel: a.st.motorVel,
        motorTrq: a.st.motorTrq, cur: a.st.cur, followingErr: a.st.followingErr,
        enabled: a.st.enabled, fault: a.st.fault, steps: a.st.steps,
      })),
    };
  }
}
