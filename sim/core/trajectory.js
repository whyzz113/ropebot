/**
 * 轨迹生成（与固件 firmware/core/trajectory.c 一致）。
 *
 * 所有运动都以「从当前位姿出发」的平滑插值（五次多项式）实现，避免速度/加速度突变：
 *   hold        悬停
 *   line        直线到目标位姿
 *   circle      空间圆 + 横滚/俯仰/偏航正弦（演示圆周工作空间与姿态跟踪）
 *   palletize   十字取放（取料位 -> 抬升 -> 转运 -> 放料位 -> 回程）
 * 输出位姿统一经过 clamp() 限幅到工作空间内。
 */
import { PI } from './core.js';

export const smoothstep5 = (t) => {
  const s = Math.min(1, Math.max(0, t));
  return s * s * s * (10 - 15 * s + 6 * s * s);
};
const dsmoothstep5 = (t) => {
  const s = Math.min(1, Math.max(0, t));
  return 30 * s * s * (1 - s) * (1 - s);
};

export class Trajectory {
  constructor(params) {
    this.p = params;
    this.reset();
  }

  reset(q) {
    const hp = this.p.controller.home_pose;
    this.q = q ? Float64Array.from(q) : Float64Array.from(hp);
    this.q0 = Float64Array.from(this.q);
    this.q1 = Float64Array.from(this.q);
    this.mode = 'hold';
    this.t = 0;
    this.t0 = 0;
    this.duration = 1;
    this.circleBlend = 0; // 0 -> 1 进入圆轨迹的平滑系数
    this.circleQ0 = Float64Array.from(this.q);
    this.pts = [];
    this.segTime = 1.5;
    this.palletIdx = 0;
  }

  /**
   * 直线到目标位姿，并按速度/加速度上限自动放慢（运动学平滑）。
   * 对 8 绳 CDPR 来说，重力＋惯性外力完全由绳拉力提供，加减速越大，
   * 所需张力越集中，对张力分配不利；因此默认按 max_trans_acc 限速。
   */
  goto(qTarget, dur = 2.0) {
    const target = this.clamp(Float64Array.from(qTarget));
    const c = this.p.controller;
    let dist = 0;
    for (let k = 0; k < 3; k++) dist = Math.max(dist, Math.abs(target[k] - this.q[k]));
    let ang = 0;
    for (let k = 3; k < 6; k++) ang = Math.max(ang, Math.abs(target[k] - this.q[k]));
    // 峰加速度 = 5.77 * d / T^2（五次多项式）；限制到 max_trans_acc / max_ang_acc
    const aMax = c.max_trans_acc ?? 1.2;
    const wMax = c.max_ang_acc ?? 6.0;
    const tAcc = Math.max(dist > 1e-9 ? Math.sqrt(5.77 * dist / aMax) : 0, ang > 1e-9 ? Math.sqrt(5.77 * ang / wMax) : 0);
    this.q0 = Float64Array.from(this.q);
    this.q1 = target;
    this.duration = Math.max(0.3, dur, tAcc);
    this.t0 = this.t;
    this.mode = 'line';
  }

  circle(cfg = this.p.trajectory) {
    this.cfg = cfg;
    this.circleBlend = 0;
    this.circleQ0 = Float64Array.from(this.q);
    this.t0 = this.t;
    this.mode = 'circle';
  }

  /** 十字取放：按 count 行排列，每行「取料 -> 抬升 -> 放置」 */
  palletize(cfg = this.p.palletizing) {
    const n = Math.max(1, Math.round(cfg.count));
    const pts = [Array.from(this.q.slice(0, 3))];
    const cur = Array.from(this.q.slice(0, 3));
    for (let i = 0; i < n; i++) {
      const zc = cfg.pick_z0 ?? 0; // 行位置沿 z 分布
      const z = (i - (n - 1) / 2) * cfg.spacing;
      const pick = [cfg.pick_x0, cfg.pick_height, z];
      const upPick = [cfg.pick_x0, cfg.transfer_height, z];
      const placeX = (cfg.spacing * (n - 1)) / 2;
      const upPlace = [placeX, cfg.transfer_height, z];
      const place = [placeX, cfg.pick_height, z];
      pts.push(pick, upPick, upPlace, place);
      // 返回转运高度，准备下一行（蛇形）
      pts.push([placeX, cfg.transfer_height, z], [cfg.pick_x0, cfg.transfer_height, z]);
    }
    this.pts = pts;
    // 按段长与加速度上限计算每段时间（五次多项式峰加速度 5.77 d / T^2）。
    // 码垛用较低的加速度（pallet_acc），因为 8 绳 CDPR 的张力分配在平台
    // 大范围横移、水平加速度突然反向时，需要足够的力余量才能保持全部绳绷紧。
    const aMax = this.p.controller.pallet_acc ?? 0.3;
    this.segTimes = [];
    for (let i = 0; i < pts.length - 1; i++) {
      let d = 0;
      for (let k = 0; k < 3; k++) d = Math.max(d, Math.abs(pts[i + 1][k] - pts[i][k]));
      this.segTimes.push(Math.max(cfg.segment_time, Math.sqrt(5.77 * d / aMax)));
    }
    this.segTime = Math.max(...this.segTimes);
    this.t0 = this.t;
    this.mode = 'palletize';
  }

  stop() {
    this.q0 = Float64Array.from(this.q);
    this.q1 = Float64Array.from(this.q);
    this.mode = 'hold';
  }

  update(t) {
    this.t = t;
    const dt = t - this.t0;
    let q = Float64Array.from(this.q);
    if (this.mode === 'line') {
      const s = smoothstep5(dt / this.duration);
      for (let k = 0; k < 6; k++) q[k] = this.q0[k] + (this.q1[k] - this.q0[k]) * s;
      if (dt >= this.duration) {
        this.q.set(this.q1);
        this.mode = 'hold';
      }
    } else if (this.mode === 'circle') {
      const g = this.cfg;
      const w = (2 * PI) / Math.max(g.period, 0.5);
      const ph = w * dt;
      // 进入圆轨迹的过渡（前 3 s）
      this.circleBlend += 0;
      const blend = smoothstep5(Math.min(1, dt / 3.0));
      const target = [
        g.xy_amplitude * Math.sin(ph),
        g.height + g.z_amplitude * Math.sin(2 * ph),
        g.z_amplitude * 0.6 * Math.sin(ph),
        g.roll_amplitude * Math.sin(ph),
        g.pitch_amplitude * Math.cos(ph),
        g.yaw_amplitude * Math.sin(0.5 * ph),
      ];
      for (let k = 0; k < 6; k++) q[k] = this.circleQ0[k] + (target[k] - this.circleQ0[k]) * blend;
    } else if (this.mode === 'palletize') {
      // 累计段时长，找到当前段
      let acc = 0;
      let idx = -1;
      for (let i = 0; i < this.segTimes.length; i++) {
        if (dt < acc + this.segTimes[i]) { idx = i; break; }
        acc += this.segTimes[i];
      }
      if (idx < 0 || this.pts.length < 2) {
        this.mode = 'hold';
      } else {
        const f = smoothstep5((dt - acc) / this.segTimes[idx]);
        const A = this.pts[idx], B = this.pts[idx + 1];
        q[0] = A[0] + (B[0] - A[0]) * f;
        q[1] = A[1] + (B[1] - A[1]) * f;
        q[2] = A[2] + (B[2] - A[2]) * f;
        q[3] = 0; q[4] = 0; q[5] = 0;
      }
    } else if (this.mode === 'demo') {
      // 关键帧序列：每帧从「上一帧终点」平滑过渡到「本帧目标」
      const keys = this.demoKeys || [];
      if (!keys.length) {
        this.mode = 'hold';
      } else {
        let acc = 0;
        let idx = -1;
        for (let i = 0; i < keys.length; i++) {
          if (dt < acc + keys[i].t) { idx = i; break; }
          acc += keys[i].t;
        }
        if (idx < 0) {
          // 序列播完
          if (this.demoLoop) {
            this.t0 = t;
            this.demoStart = Float64Array.from(this.q);
            idx = 0;
            acc = 0;
            this.demoIdx = 0;
            this.demoAcc = 0;
          } else {
            this.mode = 'hold';
            this.demoIdx = keys.length;
          }
        }
        if (idx >= 0) {
          this.demoIdx = idx;
          this.demoAcc = dt - acc;
          const f = smoothstep5((dt - acc) / keys[idx].t);
          // 段起点固定为「上一关键帧」，不能每帧更新（否则 A 跟着 B 走，
          // 目标会直接跳到段末）。idx 0 用序列开始时的位姿。
          const prevK = idx === 0 ? null : keys[idx - 1];
          const A = idx === 0 ? this.demoStart
            : [prevK.p[0], prevK.p[1], prevK.p[2], prevK.rpy[0], 0, prevK.rpy[2]];
          const B = keys[idx];
          q[0] = A[0] + (B.p[0] - A[0]) * f;
          q[1] = A[1] + (B.p[1] - A[1]) * f;
          q[2] = A[2] + (B.p[2] - A[2]) * f;
          q[3] = A[3] + (B.rpy[0] - A[3]) * f;
          q[4] = 0;                       // 俯仰不可控，恒为 0
          q[5] = A[5] + (B.rpy[2] - A[5]) * f;
        }
      }
    } else if (this.mode === 'seq') {
      // 用户路径点序列：按累计时长定位当前段，段内五次多项式插值
      const pts = this.seqPts || [];
      const times = this.seqTimes || [];
      if (!pts.length || times.length !== pts.length) {
        this.mode = 'hold';
      } else {
        let acc = 0;
        let idx = -1;
        for (let i = 0; i < times.length; i++) {
          if (dt < acc + times[i]) { idx = i; break; }
          acc += times[i];
        }
        if (idx < 0) {
          if (this.seqLoop) {
            // 循环：回到起点重新走
            this.t0 = t;
            this.seqStart = Float64Array.from(this.q);
            this.seqPrev = Float64Array.from(this.q);
            this.seqIdx = 0;
            this.seqAcc = 0;
            idx = 0;
            acc = 0;
          } else {
            this.mode = 'hold';
            this.seqIdx = pts.length;
            q = Float64Array.from(pts[pts.length - 1].q);
            this.q = this.clamp(q);
            return this.q;
          }
        }
        this.seqIdx = idx;
        this.seqAcc = dt - acc;
        const f = smoothstep5((dt - acc) / times[idx]);
        // 段起点 = 上一段的终点（固定不变）；idx 0 时用序列开始时的位姿。
        // 注意：不能用「每帧更新的 prev」，否则 A 会跟着 B 一起变，q 在第
        // 二帧就直接跳到段末（曾经的真实 bug）。
        const A = idx === 0 ? this.seqStart : pts[idx - 1].q;
        const B = pts[idx].q;
        for (let k = 0; k < 6; k++) q[k] = A[k] + (B[k] - A[k]) * f;
      }
    }
    this.q = this.clamp(q);
    return this.q;
  }

  /**
   * 演示序列：把视频里的典型运动拆成「关键帧 + 平滑插值」。
   *
   * @param {Array<{p:number[], rpy:number[], t:number}>} keys
   *   关键帧列表，t 为该帧的持续时间（秒）。rpy 用 [roll, pitch, yaw]。
   * @param {object} opts { loop:boolean }
   *
   * 严格 8 角拓扑下俯仰（pitch）不可控，这里会把 pitch 分量强制归零，
   * 保证演示过程中张力始终可行，不会中途「掉下去」。
   */
  demo(keys, opts = {}) {
    this.demoKeys = keys.map((k) => ({
      p: [k.p?.[0] ?? 0, k.p?.[1] ?? this.q[1], k.p?.[2] ?? 0],
      rpy: [k.rpy?.[0] ?? 0, 0, k.rpy?.[2] ?? 0],   // pitch 强制 0
      t: Math.max(0.2, k.t ?? 1.0),
    }));
    this.demoLoop = !!opts.loop;
    this.demoStart = Float64Array.from(this.q);
    this.t0 = this.t;
    this.mode = 'demo';
    this.demoIdx = 0;
    this.demoAcc = 0;
  }

  /** 当前序列总时长 */
  demoDuration() {
    return (this.demoKeys || []).reduce((s, k) => s + k.t, 0);
  }

  /** 当前处在第几段 / 共几段（给界面显示） */
  demoProgress() {
    if (this.mode !== 'demo' || !this.demoKeys) return null;
    return { idx: this.demoIdx, total: this.demoKeys.length, segT: this.demoAcc };
  }

  /**
   * 用户输入的路径点序列：依次经过每个坐标，每段用五次多项式平滑。
   *
   * 与 demo() 的区别：demo 是视频复现、强制 pitch=0；seq 是用户编程，
   * 6 个分量都允许（pitch 虽然在严格 8 角拓扑下不可控，但保留通道以便
   * 换成 edge 拓扑后可直接用）。
   *
   * @param {Array<{p:number[], rpy:number[], t:number}>} pts
   *   路径点列表，t 为「走到该点」所用时间（秒）；省略则按运动学上限自动算。
   * @param {object} opts { loop, speed_scale }
   */
  moveSeq(pts, opts = {}) {
    const list = (pts || []).map((k) => ({
      q: [
        k.p?.[0] ?? this.q[0],
        k.p?.[1] ?? this.q[1],
        k.p?.[2] ?? this.q[2],
        k.rpy?.[0] ?? 0,
        k.rpy?.[1] ?? 0,
        k.rpy?.[2] ?? 0,
      ],
      t: k.t ?? null,
    }));
    this.seqPts = list;
    this.seqLoop = !!opts.loop;
    this.seqSpeed = Math.max(0.1, opts.speed_scale ?? 1);
    this.seqStart = Float64Array.from(this.q);
    this.seqPrev = Float64Array.from(this.q);
    this.seqTimes = [];
    // 逐段计算时长：未指定时按 max_trans_acc / max_ang_acc 反推（五次多项式
    // 峰加速度 5.77 d / T²），并保证峰值速度 1.875 d / T 不超限。
    const c = this.p.controller;
    const aMax = (c.max_trans_acc ?? 1.2) * 0.5;   // 留一半余量给张力分配
    const wMax = (c.max_ang_acc ?? 6.0) * 0.5;
    const vMax = (c.max_trans_vel ?? 0.45) * 0.6;
    const wvMax = (c.max_ang_vel ?? 1.0) * 0.6;
    let from = this.seqStart;
    for (const k of list) {
      if (k.t != null) {
        this.seqTimes.push(Math.max(0.2, k.t));
      } else {
        // 平移与姿态各取「最大单轴变化量」，分别按时长/速度上限反推段时长
        let dTrans = 0, dAng = 0;
        for (let i = 0; i < 3; i++) dTrans = Math.max(dTrans, Math.abs(k.q[i] - from[i]));
        for (let i = 3; i < 6; i++) dAng = Math.max(dAng, Math.abs(k.q[i] - from[i]));
        const byAcc = Math.max(
          dTrans > 1e-9 ? Math.sqrt(5.77 * dTrans / aMax) : 0,
          dAng > 1e-9 ? Math.sqrt(5.77 * dAng / wMax) : 0,
        );
        const byVel = Math.max(
          dTrans > 1e-9 ? (1.875 * dTrans) / vMax : 0,
          dAng > 1e-9 ? (1.875 * dAng) / wvMax : 0,
        );
        this.seqTimes.push(Math.max(0.2, Math.max(byAcc, byVel)) / this.seqSpeed);
      }
      from = k.q;
    }
    this.t0 = this.t;
    this.mode = 'seq';
    this.seqIdx = 0;
    this.seqAcc = 0;
  }

  /** 路径点进度（给界面显示） */
  seqProgress() {
    if (this.mode !== 'seq' || !this.seqPts) return null;
    return { idx: this.seqIdx, total: this.seqPts.length, segT: this.seqAcc };
  }

  /** 路径点序列总时长 */
  seqDuration() {
    return (this.seqTimes || []).reduce((s, v) => s + v, 0);
  }

  clamp(q) {
    const p = this.p;
    const c = p.controller;
    const m = p.platform;
    const [mx, my, mz] = c.workspace_margins;
    const cv = (v, lo, hi) => Math.min(hi, Math.max(lo, v));
    const xl = p.winders.half_spacing_x - m.half_size_x - mx;
    const zl = p.winders.half_spacing_z - m.half_size_z - mz;
    q[0] = cv(q[0], -xl, xl);
    q[2] = cv(q[2], -zl, zl);
    // 竖直范围：平台必须落在「上锚点之下、下锚点之上」，
    // 这样上组绳能向上拉、下组绳能向下拉，形成双向力封闭。
    //
    // 旧写法用 `winders.lower_y - 0.05` 作为上限，隐含假设「平台在下锚点之下」。
    // 严格 8 角拓扑把下组锚点放到了框架底部（lower_y = 0.10），
    // 那个假设会让上限变成 0.05，把平台直接压到地面 —— 所以改成用上锚点定上限。
    const upY = p.winders.upper_y ?? 1.0;
    const loY = p.winders.lower_y ?? 0.0;
    const yLo = (c.floor_y ?? 0.05) + 0.03 + my;
    const yHi = Math.max(yLo + 0.05, Math.min(upY - 0.06, (upY + loY) / 2 + 0.25));
    q[1] = cv(q[1], yLo, yHi);
    q[3] = cv(q[3], -c.limits.roll_rad, c.limits.roll_rad);
    q[4] = cv(q[4], -c.limits.pitch_rad, c.limits.pitch_rad);
    q[5] = cv(q[5], -c.limits.yaw_rad, c.limits.yaw_rad);
    return q;
  }
}
