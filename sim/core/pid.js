/**
 * 任务空间 PID 控制器（6 自由度：x/y/z + 三个姿态角）。
 *
 * 为什么是「任务空间」而不是「关节空间」：
 *   8 绳冗余驱动的平台位姿由绳长唯一确定，但绳长误差到姿态误差的映射是
 *   病态的（严格 8 角拓扑的 My 行甚至可以恒为 0）。在关节空间各自闭环时，
 *   姿态误差没有任何通道能被纠正；在任务空间闭环则由 PID 直接产出
 *   「需要的合力/合力矩」，再交给张力分配去解，姿态误差天然被纳入。
 *
 * 输出是 6 维力旋量 w_pid = [Fx, Fy, Fz, Mx, My, Mz]（N 与 N·m），
 * 由 controller.js 叠加到重力/惯性前馈上，一起送进 allocateT()。
 * 因此 PID 增益的量纲是：
 *   平移：N/m、N/(m·s)、N·s/m
 *   姿态：N·m/rad、N·m/(rad·s)、N·m·s/rad
 *
 * 稳定性的经验界限：任务空间刚度 K ≈ Jᵀ·diag(k)·J（约 5 kN/m、
 * 10~20 N·m/rad），闭环增益乘积 K⁻¹·Kp 必须 < 1，否则会和位置环
 * 里的弹性预补偿耦合出振荡。默认值取在该界限的 1/5 左右。
 */

/** 把值限制在 [-lim, lim] */
const clampAbs = (v, lim) => Math.max(-lim, Math.min(lim, v));

export class PidAxis {
  /**
   * @param {number} kp 比例增益
   * @param {number} ki 积分增益
   * @param {number} kd 微分增益
   * @param {number} outMax 输出限幅（绝对值）
   */
  constructor(kp, ki, kd, outMax) {
    this.kp = kp;
    this.ki = ki;
    this.kd = kd;
    this.outMax = outMax;
    this.reset();
  }

  reset() {
    this.integ = 0;
    this.prevErr = null;
    this.dFilt = 0;
    this.lastOut = 0;
    this.saturated = false;
  }

  /**
   * @param {number} err 目标 - 实测
   * @param {number} dt 控制周期（s）
   * @param {number} dCutoffHz 微分项低通截止频率（Hz），抑制逐周期噪声
   */
  step(err, dt, dCutoffHz = 8) {
    const dtc = Math.max(dt, 1e-6);
    // 微分：对误差做一阶低通后再差分，避免编码器量化噪声被放大
    let dRaw = 0;
    if (this.prevErr !== null) dRaw = (err - this.prevErr) / dtc;
    const wc = 2 * Math.PI * dCutoffHz;
    const a = dtc / (dtc + 1 / wc);
    this.dFilt += a * (dRaw - this.dFilt);
    this.prevErr = err;

    // 积分：先算，若输出饱和且误差同向增长则回退（conditional integration 抗饱和）
    const pTerm = this.kp * err;
    const dTerm = this.kd * this.dFilt;
    const iCandidate = this.integ + this.ki * err * dtc;
    let out = pTerm + iCandidate + dTerm;

    if (Math.abs(out) > this.outMax) {
      // 饱和：只有在误差会把输出推得更远时才冻结积分
      const over = out > 0 ? 1 : -1;
      const pushingOut = Math.sign(err) === over;
      if (!pushingOut) this.integ = iCandidate;
      out = clampAbs(pTerm + this.integ + dTerm, this.outMax);
      this.saturated = true;
    } else {
      this.integ = iCandidate;
      this.saturated = false;
    }
    this.lastOut = out;
    return out;
  }
}

export class Pid6 {
  /**
   * @param {object} cfg controller.pid 参数块
   * @param {object} params 全局参数（用于算重力量级做限幅）
   */
  constructor(cfg, params) {
    this.cfg = cfg || {};
    const p = params;
    // 输出限幅：不允许 PID 单独要求超过重力一定倍数的力，否则会解不出正张力
    const gW = (p.platform.mass + 0) * p.platform.gravity;
    const fMax = (this.cfg.force_limit_ratio ?? 0.8) * gW;
    const mMax = (this.cfg.moment_limit_ratio ?? 0.8) * gW * 0.15; // 力臂按 0.15 m 估
    const t = this.cfg.trans || {};
    const r = this.cfg.rot || {};
    this.axes = [
      new PidAxis(t.kp ?? 400, t.ki ?? 40, t.kd ?? 30, fMax),
      new PidAxis(t.kp ?? 400, t.ki ?? 40, t.kd ?? 30, fMax),
      new PidAxis(t.kp ?? 400, t.ki ?? 40, t.kd ?? 30, fMax),
      new PidAxis(r.kp ?? 6, r.ki ?? 1.2, r.kd ?? 0.8, mMax),
      new PidAxis(r.kp ?? 6, r.ki ?? 1.2, r.kd ?? 0.8, mMax),
      new PidAxis(r.kp ?? 6, r.ki ?? 1.2, r.kd ?? 0.8, mMax),
    ];
    this.dCutoff = this.cfg.d_filter_hz ?? 8;
    this.enabled = this.cfg.enable ?? false;
    this.out = new Float64Array(6);
    this.err = new Float64Array(6);
  }

  reset() {
    for (const a of this.axes) a.reset();
    this.out.fill(0);
    this.err.fill(0);
  }

  /** 运行时可调增益（界面滑块） */
  setGains(kp, ki, kd, group = 'trans') {
    const g = this.cfg[group] || (this.cfg[group] = {});
    if (kp !== undefined) g.kp = kp;
    if (ki !== undefined) g.ki = ki;
    if (kd !== undefined) g.kd = kd;
    const idx = group === 'trans' ? [0, 1, 2] : [3, 4, 5];
    for (const i of idx) {
      if (kp !== undefined) this.axes[i].kp = kp;
      if (ki !== undefined) this.axes[i].ki = ki;
      if (kd !== undefined) this.axes[i].kd = kd;
    }
  }

  /**
   * @param {number} dt 控制周期
   * @param {ArrayLike<number>} qTarget 目标位姿
   * @param {ArrayLike<number>} qMeas 实测位姿
   * @returns {Float64Array} 6 维力旋量
   */
  step(dt, qTarget, qMeas) {
    if (!this.enabled) {
      this.out.fill(0);
      return this.out;
    }
    for (let k = 0; k < 6; k++) {
      const e = qTarget[k] - qMeas[k];
      this.err[k] = e;
      this.out[k] = this.axes[k].step(e, dt, this.dCutoff);
    }
    return this.out;
  }

  /** 诊断信息（界面显示） */
  diag() {
    return {
      enabled: this.enabled,
      err: Array.from(this.err).map((v) => +v.toFixed(5)),
      out: Array.from(this.out).map((v) => +v.toFixed(2)),
      integ: this.axes.map((a) => +a.integ.toFixed(3)),
    };
  }
}
