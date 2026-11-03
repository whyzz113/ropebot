/**
 * 仿真引擎：把「动力学 + 控制器 + 轨迹 + 电机模型」按真实的多速率结构组合起来。
 *
 * 多速率（与硬件一致）：
 *   物理积分      2000 Hz（0.5 ms）
 *   电机/驱动器   1000 Hz（1 ms）
 *   控制器        1000 Hz（1 ms）
 *   轨迹/目标位姿  100 Hz（10 ms）
 *
 * 张力测量用「传感器」语义：真实状态下每根绳张力，作为控制器输入。
 * 本文件同时被浏览器 worker 和 Node 命令行使用。
 */
import { CdprSim } from './model.js';
import { Controller } from './controller.js';
import { Trajectory } from './trajectory.js';
import { buildModel, structure } from './core.js';

export class Engine {
  /**
   * @param {object} params 读入并合并后的参数
   * @param {object} opts { fixedStep: 推荐 1/2000 }
   */
  constructor(params, opts = {}) {
    this.p = params;
    this.model = buildModel(params);
    this.sim = new CdprSim(params, this.model);
    this.ctl = new Controller(params, this.model);
    this.traj = new Trajectory(params);
    this.step = opts.fixedStep ?? 1 / (params.loop.physics_rate_hz || 2000);
    this.n = this.model.nCables;
    this.h = this.step;
    // 控制器/驱动器以物理步为整数倍运行，这里全部是每物理步跑一次的控制假设：
    this.ctlEvery = Math.max(1, Math.round((params.loop.physics_rate_hz / (params.loop.controller_rate_hz || 1000))));
    this.drvEvery = Math.max(1, Math.round((params.loop.physics_rate_hz / (params.loop.driver_rate_hz || 1000))));
    this.wpEvery = Math.max(1, Math.round((params.loop.physics_rate_hz / (params.loop.waypoint_rate_hz || 100))));
    this.logEvery = Math.max(1, Math.round((params.loop.physics_rate_hz / (params.loop.log_rate_hz || 1000))));
    this.counter = 0;
    this.t = 0;
    this.qTarget = Float64Array.from(params.controller.home_pose);
    this.measT = new Float64Array(this.n).fill(0);
    this.last = null;
    // 记录历史（给曲线用，环形缓冲）
    this.history = [];
    this.historyMax = 4000;
    this.lastSnap = null;
    this.cmds = null;
    this.ready = false;
    this.init();
  }

  init() {
    const p = this.p;
    this.sim.reset();
    this.ctl.reset();
    this.traj.reset(this.sim.x);
    // 让张力从 0 平滑建立：给卷筒一个小的收绳量，使绳长略短于几何长度
    this.sim.ctlEnabled = true;
    this.ready = true;
  }

  /** 设置附加负载（kg） */
  setLoad(kg) {
    this.ctl.setLoad(kg);
  }

  /** 高位接口：预设位姿/圆轨迹/码垛/悬停 */
  gotoPose(pose, dur = 2.5) {
    const q = Float64Array.from(pose);
    this.traj.goto(q, dur);
  }

  gotoNamed(name, dur = 2.5) {
    const pose = this.p.poses.find((s) => s.name === name);
    if (!pose) return false;
    this.traj.goto(Float64Array.from([pose.p[0], pose.p[1], pose.p[2], pose.rpy[0], pose.rpy[1], pose.rpy[2]]), dur);
    return true;
  }

  startCircle() {
    this.traj.circle(this.p.trajectory);
  }

  startPalletize() {
    this.traj.palletize(this.p.palletizing);
  }

  /**
   * 演示序列：按关键帧播放一组运动（对应参考视频里的典型动作）。
   * @param {string} name 序列名，见 p.demos
   * @param {boolean} loop 是否循环
   */
  startDemo(name, loop = false) {
    const keys = this.p.demos?.[name];
    if (!keys || !keys.length) return false;
    this.traj.demo(keys, { loop });
    return true;
  }

  /** 当前演示进度（给界面显示） */
  demoProgress() {
    return this.traj.demoProgress ? this.traj.demoProgress() : null;
  }

  /**
   * 用户路径点序列：依次经过给定坐标。
   * @param {Array<{p:number[], rpy:number[], t:number}>} pts
   * @param {object} opts { loop, speed_scale }
   */
  startSeq(pts, opts = {}) {
    if (!pts || !pts.length) return false;
    this.traj.moveSeq(pts, opts);
    return true;
  }

  seqProgress() {
    return this.traj.seqProgress ? this.traj.seqProgress() : null;
  }

  seqDuration() {
    return this.traj.seqDuration ? this.traj.seqDuration() : 0;
  }

  /** 开启/关闭任务空间 PID */
  enablePid(on) {
    if (!this.ctl.pid) return false;
    this.ctl.pid.enabled = !!on;
    if (!on) this.ctl.pid.reset();
    this.p.controller.pid.enable = !!on;
    return true;
  }

  /** 运行时调整 PID 增益：group = 'trans' | 'rot' */
  setPidGains(group, kp, ki, kd) {
    if (!this.ctl.pid) return false;
    this.ctl.pid.setGains(kp, ki, kd, group);
    return true;
  }

  pidDiag() {
    return this.ctl.pid ? this.ctl.pid.diag() : null;
  }

  stop() {
    this.traj.stop();
  }

  /** 推进一个物理步 */
  advance() {
    const p = this.p;
    // 1) 轨迹（100 Hz 即可，这里每个物理步更新但保持平滑）
    this.qTarget = Float64Array.from(this.traj.update(this.t));
    // 2) 控制器（1 kHz）
    if (this.counter % this.ctlEvery === 0) {
      const meas = { contact: this.sim.contact, q: Array.from(this.sim.x) };
      this.cmds = this.ctl.step(this.ctlEvery * this.h, this.qTarget, this.measT, meas);
      this.sim.applyCommands(this.cmds, this.ctl.l0);
    }
    // 3) 驱动器闭环（1 kHz）
    if (this.counter % this.drvEvery === 0) {
      for (let i = 0; i < this.n; i++) this.sim.acts[i].stepDriver(this.drvEvery * this.h);
    }
    // 4) 物理
    this.sim.step(this.h);
    this.measT = Float64Array.from(this.sim.tension);
    // 5) 记录
    if (this.counter % this.logEvery === 0) {
      const x = this.sim.x;
      const T = this.sim.tension;
      this.history.push({
        t: +this.t.toFixed(4),
        x: [x[0], x[1], x[2]].map((v) => +v.toFixed(4)),
        T: Array.from(T).map((v) => +v.toFixed(2)),
        qT: Array.from(this.qTarget.slice(0, 3)).map((v) => +v.toFixed(4)),
      });
      if (this.history.length > this.historyMax) this.history.shift();
    }
    this.t += this.h;
    this.counter++;
  }

  run(seconds) {
    const steps = Math.round(seconds / this.h);
    for (let k = 0; k < steps; k++) this.advance();
    return this.snapshot();
  }

  /** 当前完整快照（给渲染与 UI） */
  snapshot() {
    const sim = this.sim;
    const st = structure(this.model, sim.x);
    const acts = sim.acts.map((a) => ({
      cablePos: +a.st.cablePos.toFixed(5),
      cableVel: +a.st.cableVel.toFixed(5),
      motorVel: +a.st.motorVel.toFixed(2),
      motorTrq: +a.st.motorTrq.toFixed(3),
      cur: +a.st.cur.toFixed(3),
      followingErr: +a.st.followingErr.toFixed(5),
      steps: a.st.steps,
      enabled: a.st.enabled,
      fault: a.st.fault,
    }));
    return {
      t: +this.t.toFixed(4),
      q: Array.from(sim.x).map((v) => +v.toFixed(5)),
      qTarget: Array.from(this.qTarget).map((v) => +v.toFixed(5)),
      l: Array.from(st.l).map((v) => +v.toFixed(5)),
      T: Array.from(sim.tension).map((v) => +v.toFixed(2)),
      Tref: Array.from(this.ctl.tRef).map((v) => +v.toFixed(2)),
      acts,
      contact: sim.contact,
      feasible: this.ctl.feasible,
      vp: sim.vp.map((v) => +v.toFixed(4)),
      w: sim.w.map((v) => +v.toFixed(4)),
      pid: this.ctl.pid ? this.ctl.pid.diag() : null,
      seq: this.traj.seqProgress ? this.traj.seqProgress() : null,
      trajMode: this.traj.mode,
    };
  }
}
