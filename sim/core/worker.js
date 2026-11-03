/**
 * 浏览器端仿真 Worker：在后台线程按实时倍率推进 Engine，避免阻塞渲染。
 *
 * 协议（与页面 UI 之间）：
 *   输入  {type:'init', params, overrides} | {type:'cmd', action, arg}
 *         {type:'load', kg} | {type:'setSimRate', value} | {type:'pause'|'resume'}
 *   输出  {type:'ready', params} | {type:'state', snapshot} | {type:'history', samples}
 *         {type:'log', level, text} | {type:'paramsError', errors}
 */
import { Engine } from './engine.js';
import { readParams } from './params.js';

let engine = null;
let paused = false;
let simRate = 1.0;
let lastWall = 0;
let accumulator = 0;
let overrides = {};
let running = false;

const post = (msg) => self.postMessage(msg);
const log = (level, text) => post({ type: 'log', level, text });

async function init(msg) {
  overrides = msg.overrides || {};
  const { params, errors, loaded } = await readParams(overrides);
  if (errors.length) post({ type: 'paramsError', errors });
  engine = new Engine(params);
  engine.run(1.5); // 上电预紧：和真实设备上电找零后的状态对齐
  post({
    type: 'ready',
    params,
    loaded,
    layout: {
      anchors: engine.model.anchors,
      platformPoints: engine.model.platformPointsLocal,
    },
  });
  log('info', `仿真核心就绪（${params.loop.physics_rate_hz} Hz 物理 / ${params.loop.controller_rate_hz} Hz 控制 / ${params.loop.driver_rate_hz} Hz 驱动）`);
  lastWall = performance.now();
  startLoop();
}

function startLoop() {
  if (running) return;
  running = true;
  const tick = () => {
    const now = performance.now();
    const dtWall = Math.min(0.1, (now - lastWall) / 1000);
    lastWall = now;
    if (engine && !paused) {
      accumulator += dtWall * simRate;
      const h = engine.h;
      let steps = Math.floor(accumulator / h);
      const maxSteps = 400; // 单帧上限，防止卡顿后追不上
      if (steps > maxSteps) {
        steps = maxSteps;
        accumulator = 0;
      } else {
        accumulator -= steps * h;
      }
      for (let k = 0; k < steps; k++) engine.advance();
      post({ type: 'state', snapshot: engine.snapshot(), simRate, paused, trajMode: engine.traj.mode });
    }
    setTimeout(tick, 8);
  };
  tick();
}

function handleCmd(msg) {
  if (!engine) return;
  const { action, arg } = msg;
  const e = engine;
  switch (action) {
    case 'goto': {
      // arg: {name} 或 {pose:[6]}
      if (arg.name) {
        const ok = e.gotoNamed(arg.name, arg.dur ?? 2.5);
        log('info', ok ? `轨迹：移动到预设位姿 ${arg.name}` : `未找到预设位姿 ${arg.name}`);
      } else {
        e.traj.goto(Float64Array.from(arg.pose), arg.dur ?? 2.5);
        log('info', `轨迹：移动到 [${arg.pose.map((v) => v.toFixed(3)).join(', ')}]`);
      }
      break;
    }
    case 'circle':
      e.startCircle();
      log('info', '轨迹：圆轨迹 + 姿态摆动');
      break;
    case 'palletize':
      e.startPalletize();
      log('info', '轨迹：码垛（十字取放）');
      break;
    case 'hold':
      e.stop();
      log('info', '轨迹：原地保持');
      break;
    case 'demo': {
      // arg: {name, loop} —— 播放参考视频里的典型动作序列
      const ok = e.startDemo(arg.name, !!arg.loop);
      if (ok) {
        const keys = e.p.demos[arg.name];
        const dur = keys.reduce((s, k) => s + k.t, 0);
        log('info', `演示「${arg.name}」：${keys.length} 段，共 ${dur.toFixed(1)} s`);
      } else {
        log('warn', `未找到演示序列 ${arg.name}`);
      }
      break;
    }
    case 'seq': {
      // arg: {points:[{p,rpy,t}], loop, speed_scale} —— 用户输入的坐标序列
      const ok = engine.startSeq(arg.points, { loop: !!arg.loop, speed_scale: arg.speed_scale });
      if (ok) {
        const n = arg.points.length;
        const dur = engine.seqDuration();
        log('info', `路径序列：${n} 个目标点，共 ${dur.toFixed(1)} s`);
      } else {
        log('warn', '路径序列为空');
      }
      break;
    }
    case 'setPid': {
      // arg: {enable} 或 {group,kp,ki,kd}
      if (arg.enable !== undefined) {
        engine.enablePid(arg.enable);
        log('info', arg.enable ? '任务空间 PID：已启用' : '任务空间 PID：已关闭');
      }
      if (arg.group) {
        engine.setPidGains(arg.group, arg.kp, arg.ki, arg.kd);
        log('info', `PID 增益 ${arg.group}: kp=${arg.kp} ki=${arg.ki} kd=${arg.kd}`);
      }
      break;
    }
    case 'reset': {
      const { params } = { params: e.p };
      const eng = new Engine(params);
      eng.run(1.5);
      engine = eng;
      log('info', '系统已复位');
      break;
    }
    case 'setLoad':
      e.setLoad(arg.kg ?? 0);
      log('info', `附加负载：${(arg.kg ?? 0).toFixed(1)} kg`);
      break;
    case 'setAlloc':
      e.p.controller.tension_alloc = arg.mode;
      log('info', `张力分配模式：${arg.mode}`);
      break;
    case 'setParam': {
      // 运行时调参（不重建仿真）：arg = {path:'a.b', value}
      setByPath(e.p, arg.path, arg.value);
      log('info', `参数 ${arg.path} = ${JSON.stringify(arg.value)}（运行时）`);
      break;
    }
    case 'getHistory':
      post({ type: 'history', samples: e.history.slice(-2000) });
      break;
    default:
      log('warn', `未知命令 ${action}`);
  }
}

function setByPath(obj, path, value) {
  const parts = path.split('.');
  let o = obj;
  for (let i = 0; i < parts.length - 1; i++) o = o[parts[i]];
  o[parts[parts.length - 1]] = value;
}

self.onmessage = async (ev) => {
  const msg = ev.data;
  try {
    if (msg.type === 'init') await init(msg);
    else if (msg.type === 'cmd') handleCmd(msg);
    else if (msg.type === 'pause') { paused = true; lastWall = performance.now(); }
    else if (msg.type === 'resume') { paused = false; lastWall = performance.now(); }
    else if (msg.type === 'setSimRate') { simRate = Math.max(0.05, Math.min(8, msg.value)); }
    else if (msg.type === 'reloadParams') {
      // 参数文件已修改：重建引擎，保持当前位姿
      const q = engine ? Float64Array.from(engine.sim.x) : null;
      const { params, errors } = await readParams(overrides);
      if (errors.length) post({ type: 'paramsError', errors });
      engine = new Engine(params);
      if (q) {
        engine.sim.reset(q);
        engine.traj.reset(q);
        engine.ctl.reset();
      }
      engine.run(1.0);
      post({ type: 'ready', params, loaded: true });
      log('info', '参数文件已重新加载');
    }
  } catch (err) {
    log('error', `Worker 异常：${err && err.message ? err.message : err}`);
  }
};
