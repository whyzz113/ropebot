/**
 * 页面主逻辑：
 *  - 仿真 Worker（JS 内置控制器） 或 STM32 固件（无板仿真） 两种数据源
 *  - WebGL2 三维场景 + 实时曲线 + 控制台
 */
import { Renderer } from './renderer.js?v=74';
import { Scene, OrbitCamera } from './scene.js?v=74';
import { TimeChart } from './charts.js?v=74';

const $ = (id) => document.getElementById(id);
const logEl = $('log');
function log(level, text) {
  const div = document.createElement('div');
  div.className = level;
  const ts = new Date().toLocaleTimeString('zh-CN', { hour12: false });
  div.textContent = `[${ts}] ${text}`;
  logEl.appendChild(div);
  while (logEl.childElementCount > 300) logEl.removeChild(logEl.firstChild);
  logEl.scrollTop = logEl.scrollHeight;
}

let renderer, scene, camera, chartT, chartL;
let params = null;
let lastSnap = null;
let source = 'js';
let worker = null;
let fw = null; // 固件运行时
let fwTimer = null;

/**
 * 固件推进节拍：固件是 1 kHz 的实时控制器，这里按仿真倍率每 20 ms 请求
 * 一批步数（默认 20 步 = 20 ms 仿真时间），使画面与固件时钟同步。
 */
function startFwPump() {
  stopFwPump();
  let acc = 0;
  let last = performance.now();
  fwTimer = setInterval(() => {
    if (!fw || source !== 'fw' || paused) {
      last = performance.now();
      return;
    }
    const now = performance.now();
    const dt = Math.min(0.2, (now - last) / 1000);
    last = now;
    acc += dt * simRate;
    // 固件步进：1 步 = 1 ms
    const steps = Math.min(120, Math.max(1, Math.round(acc * 1000)));
    acc -= steps / 1000;
    fw.send({ type: 'step', n: steps });
  }, 20);
}

function stopFwPump() {
  if (fwTimer) {
    clearInterval(fwTimer);
    fwTimer = null;
  }
}
let paused = false;
let simRate = 1.0;
let frameTimes = [];

// ---------------------------------------------------------------------------
// 启动
// ---------------------------------------------------------------------------
async function boot() {
  bootCharts();
  try {
    renderer = new Renderer($('gl'));
    log('info', 'WebGL2 渲染器就绪（无第三方依赖，离线可用）');
  } catch (e) {
    log('error', e.message);
    $('runState').textContent = 'WebGL2 不可用';
    $('runState').className = 'pill err';
    return;
  }
  camera = new OrbitCamera($('gl'));
  startWorker();
  requestAnimationFrame(frame);
  bindUi();
}

function bootCharts() {
  chartT = new TimeChart($('chartT'), { count: 8, windowSec: 10, title: '绳索张力 T (N)' });
  chartL = new TimeChart($('chartL'), { count: 8, windowSec: 10, title: '绳索长度 l (m)' });
  const labels = Array.from({ length: 8 }, (_, i) => `#${i + 1}${i % 2 === 0 ? '上' : '下'}`);
  $('legendT').innerHTML = chartT.legendHtml(labels);
  $('legendL').innerHTML = chartL.legendHtml(labels);
  for (const [el, ch] of [[$('legendT'), chartT], [$('legendL'), chartL]]) {
    el.addEventListener('click', (ev) => {
      const span = ev.target.closest('.lg');
      if (!span) return;
      ch.toggle(Number(span.dataset.i));
      span.classList.toggle('off');
    });
  }
}

function startWorker() {
  // 带版本号，避免浏览器复用旧版 worker（改了 core/ 之后必须重新加载）
  worker = new Worker(new URL('../sim/core/worker.js?v=74', import.meta.url), { type: 'module' });
  worker.onmessage = (ev) => onWorkerMessage(ev.data);
  worker.onerror = (e) => log('error', `仿真 Worker 错误：${e.message}`);
  worker.postMessage({ type: 'init' });
  $('runState').textContent = '仿真核心加载中…';
}

function onWorkerMessage(msg) {
  switch (msg.type) {
    case 'ready':
      params = msg.params;
      scene = new Scene(renderer, params);
      $('runState').textContent = '运行中（JS 仿真）';
      $('runState').className = 'pill ok';
      $('tmin').textContent = params.cables.t_min.toFixed(1);
      $('tmax').textContent = params.cables.t_max.toFixed(1);
      buildPoseButtons(params.poses);
      log('info', `参数：平台 ${params.platform.mass} kg，卷筒 R=${(params.winders.drum_radius * 1000).toFixed(0)} mm，减速比 ${params.actuators.gear_ratio}，细分 ${params.actuators.driver_microstep}`);
      break;
    case 'state':
      if (source !== 'js') return;
      lastSnap = msg.snapshot;
      lastSnap.trajMode = msg.trajMode;
      break;
    case 'log':
      log(msg.level, msg.text);
      break;
    case 'paramsError':
      for (const e of msg.errors) log('warn', `参数：${e}`);
      break;
    case 'history':
      break;
    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// 渲染循环
// ---------------------------------------------------------------------------
let chartAcc = 0;
function frame(now) {
  requestAnimationFrame(frame);
  if (frameTimes.push(now) > 60) frameTimes.shift();
  renderer.resize();
  // 相机按画布的 CSS 尺寸计算纵横比，保证画面不变形
  const cw = $('gl').clientWidth || 1;
  const ch = $('gl').clientHeight || 1;
  camera.update(cw / ch);
  if (scene) {
    scene.update(lastSnap);
    scene.render(camera);
  } else {
    renderer.begin([0.06, 0.07, 0.09]);
  }
  if (lastSnap) {
    updateHud(lastSnap);
    updateTable(lastSnap);
    updatePidDiag(lastSnap);
    if (now - chartAcc > 50) {
      chartAcc = now;
      chartT.push(lastSnap.t, lastSnap.T);
      chartL.push(lastSnap.t, lastSnap.l);
    }
  }
  if (frameTimes.length > 2) {
    const dt = (frameTimes[frameTimes.length - 1] - frameTimes[0]) / (frameTimes.length - 1);
    $('fps').textContent = `${(1000 / Math.max(1, dt)).toFixed(0)} fps`;
  }
  chartT.draw();
  chartL.draw();
}

function updateHud(s) {
  const f = (v, d = 3) => v.toFixed(d);
  $('simClock').textContent = `t = ${f(s.t)} s`;
  $('hudPose').textContent = `x=${f(s.q[0])} y=${f(s.q[1])} z=${f(s.q[2])} | r=${f(s.q[3], 2)} p=${f(s.q[4], 2)} yaw=${f(s.q[5], 2)} (m, rad)`;
  $('hudTarget').textContent = `x=${f(s.qTarget[0])} y=${f(s.qTarget[1])} z=${f(s.qTarget[2])} | r=${f(s.qTarget[3], 2)} p=${f(s.qTarget[4], 2)} yaw=${f(s.qTarget[5], 2)}`;
  const fmin = Math.min(...s.T);
  const fe = $('hudFeasible');
  if (s.feasible && fmin > 0) {
    fe.textContent = `张力分配可行（Tmin=${fmin.toFixed(1)} N）`;
    fe.style.color = 'var(--ok)';
  } else {
    // 严格 8 角拓扑下这不是「参数没调好」，而是几何退化（A 的 My 行恒为 0）。
    // 把原因写在提示里，免得反复调参。
    const corner = params && params.winders && params.winders.topology === 'corner';
    fe.textContent = corner
      ? `不可行（Tmin=${fmin.toFixed(1)} N）· 严格 8 角拓扑：该姿态的力矩在 A 的零空间里`
      : `不可行！Tmin=${fmin.toFixed(1)} N（有绳松弛）`;
    fe.style.color = 'var(--err)';
  }
}

let tableAcc = 0;
function updateTable(s) {
  if (!s) return;
  const tbody = $('motorTable').querySelector('tbody');
  const rows = [];
  for (let i = 0; i < 8; i++) {
    const a = s.acts[i];
    const warn = a.fault ? 'err' : (Math.abs(a.cur) > 4 ? 'warn' : 'ok');
    const st = a.fault ? 'ALM' : (Math.abs(a.cur) > 4 ? '高电流' : (Math.abs(a.motorVel) < 0.5 ? '保持' : '运转'));
    rows.push(`<tr>
      <td>#${i + 1}${i % 2 === 0 ? '↑' : '↓'}</td>
      <td>${(s.T[i] ?? 0).toFixed(1)}</td>
      <td class="muted">${(s.Tref[i] ?? 0).toFixed(1)}</td>
      <td>${(s.l[i] ?? 0).toFixed(4)}</td>
      <td>${(((s.l[i] ?? 0) - (s.lRef0 ? s.lRef0[i] : s.l[i])) * 1000).toFixed(2)}</td>
      <td>${a.steps}</td>
      <td>${(a.motorVel * 60 / (2 * Math.PI)).toFixed(0)}</td>
      <td>${a.cur.toFixed(2)}</td>
      <td class="st-${warn}">${st}</td>
    </tr>`);
  }
  tbody.innerHTML = rows.join('');
}

// ---------------------------------------------------------------------------
// 目标坐标 / 路径点 / PID
// ---------------------------------------------------------------------------
/** 用户输入的路径点：每项 {p:[x,y,z], rpy:[r,p,y]} */
let pathPts = [];

function readPtInputs() {
  const num = (id, d) => {
    const v = Number($(id).value);
    return Number.isFinite(v) ? v : d;
  };
  return {
    p: [num('ptX', 0), num('ptY', 0.6), num('ptZ', 0)],
    rpy: [num('ptR', 0), num('ptP', 0), num('ptW', 0)],
  };
}

function writePtInputs(pt) {
  $('ptX').value = pt.p[0].toFixed(3);
  $('ptY').value = pt.p[1].toFixed(3);
  $('ptZ').value = pt.p[2].toFixed(3);
  $('ptR').value = pt.rpy[0].toFixed(3);
  $('ptP').value = pt.rpy[1].toFixed(3);
  $('ptW').value = pt.rpy[2].toFixed(3);
}

function renderPathList() {
  const box = $('ptList');
  box.innerHTML = '';
  pathPts.forEach((pt, i) => {
    const row = document.createElement('div');
    row.className = 'ptRow';
    const txt = document.createElement('span');
    txt.textContent =
      `${i + 1}. (${pt.p.map((v) => v.toFixed(2)).join(', ')})` +
      ` rpy(${pt.rpy.map((v) => v.toFixed(2)).join(', ')})`;
    txt.style.cursor = 'pointer';
    txt.title = '点击填回输入框';
    txt.addEventListener('click', () => writePtInputs(pt));
    const del = document.createElement('span');
    del.className = 'del';
    del.textContent = '×';
    del.title = '删除这个点';
    del.addEventListener('click', () => {
      pathPts.splice(i, 1);
      renderPathList();
    });
    row.appendChild(txt);
    row.appendChild(del);
    box.appendChild(row);
  });
}

/** 把路径点发给仿真核心（单点也用 seq，只是只有一个目标） */
function sendSeq(pts) {
  if (source === 'fw') {
    log('warn', '坐标规划目前只在 JS 仿真中可用，请把数据源切回 JS 仿真');
    return;
  }
  const list = pts.map((pt) => ({ p: pt.p.slice(), rpy: pt.rpy.slice() }));
  worker.postMessage({
    type: 'cmd',
    action: 'seq',
    arg: { points: list, loop: $('seqLoop').checked, speed_scale: Number($('seqSpeed').value) },
  });
}

function bindPlanUi() {
  $('btnAddPt').addEventListener('click', () => {
    pathPts.push(readPtInputs());
    renderPathList();
    log('info', `加入路径点 #${pathPts.length}`);
  });
  $('btnGoPt').addEventListener('click', () => {
    const pt = readPtInputs();
    sendSeq([pt]);
    log('info', `单点移动 → (${pt.p.map((v) => v.toFixed(3)).join(', ')})`);
  });
  $('btnRunSeq').addEventListener('click', () => {
    // 把输入框里当前的值也作为一个点追加在末尾（如果列表为空就只走这一个）
    if (pathPts.length === 0) {
      pathPts.push(readPtInputs());
      renderPathList();
    }
    sendSeq(pathPts);
    log('info', `开始走路径：${pathPts.length} 个点`);
  });
  $('btnClearPt').addEventListener('click', () => {
    pathPts = [];
    renderPathList();
  });
  $('seqSpeed').addEventListener('input', (e) => {
    $('seqSpeedVal').textContent = `${Number(e.target.value).toFixed(1)}×`;
  });

  $('pidEnable').addEventListener('change', (e) => {
    if (source === 'fw') {
      log('warn', 'PID 目前只在 JS 仿真中可用');
      e.target.checked = false;
      return;
    }
    worker.postMessage({ type: 'cmd', action: 'setPid', arg: { enable: e.target.checked } });
  });
  $('btnPidApply').addEventListener('click', () => {
    if (source === 'fw') {
      log('warn', 'PID 目前只在 JS 仿真中可用');
      return;
    }
    const num = (id) => Number($(id).value);
    worker.postMessage({
      type: 'cmd', action: 'setPid',
      arg: { group: 'trans', kp: num('pidTkp'), ki: num('pidTki'), kd: num('pidTkd') },
    });
    worker.postMessage({
      type: 'cmd', action: 'setPid',
      arg: { group: 'rot', kp: num('pidRkp'), ki: num('pidRki'), kd: num('pidRkd') },
    });
    log('info', 'PID 增益已下发');
  });
}

/** 把快照里的 PID 诊断显示出来 */
function updatePidDiag(s) {
  const el = $('pidDiag');
  if (!s || !s.pid) return;
  if (!s.pid.enabled) {
    el.textContent = 'PID 未启用';
    return;
  }
  const e = s.pid.err, o = s.pid.out;
  el.textContent =
    `误差 xyz=(${e[0].toFixed(3)}, ${e[1].toFixed(3)}, ${e[2].toFixed(3)})` +
    ` rpy=(${e[3].toFixed(3)}, ${e[4].toFixed(3)}, ${e[5].toFixed(3)})` +
    ` · 输出 F=(${o[0].toFixed(1)}, ${o[1].toFixed(1)}, ${o[2].toFixed(1)}) N`;
}

// ---------------------------------------------------------------------------
// UI
// ---------------------------------------------------------------------------
function buildPoseButtons(poses) {
  const box = $('poseBtns');
  box.innerHTML = '';
  for (const pose of poses) {
    const b = document.createElement('button');
    b.textContent = pose.name;
    b.title = `目标：x=${pose.p[0]} y=${pose.p[1]} z=${pose.p[2]}, rpy=[${pose.rpy.join(', ')}]`;
    b.addEventListener('click', () => {
      if (source === 'fw' && fw) fw.send({ type: 'goto', q: [...pose.p, ...pose.rpy] });
      else worker.postMessage({ type: 'cmd', action: 'goto', arg: { name: pose.name } });
    });
    box.appendChild(b);
  }
}

function bindUi() {
  $('btnPause').addEventListener('click', () => {
    paused = !paused;
    $('btnPause').textContent = paused ? '继续' : '暂停';
    if (source === 'fw' && fw) fw.send({ type: 'pause', value: paused });
    else worker.postMessage({ type: paused ? 'pause' : 'resume' });
  });
  $('btnReset').addEventListener('click', () => {
    if (source === 'fw' && fw) fw.restart();
    else worker.postMessage({ type: 'cmd', action: 'reset' });
    log('info', '请求复位（回到 home 位姿）');
  });
  $('btnClearLog').addEventListener('click', () => { logEl.innerHTML = ''; });

  $('simRate').addEventListener('input', (e) => {
    simRate = Number(e.target.value);
    $('simRateVal').textContent = `${simRate.toFixed(1)}×`;
    $('rateBadge').textContent = `×${simRate.toFixed(1)}`;
    if (source === 'fw' && fw) fw.send({ type: 'rate', value: simRate });
    else worker.postMessage({ type: 'setSimRate', value: simRate });
  });

  for (const b of document.querySelectorAll('[data-traj]')) {
    b.addEventListener('click', () => {
      const kind = b.dataset.traj;
      if (source === 'fw' && fw) fw.send({ type: kind === 'circle' ? 'circle' : kind === 'palletize' ? 'pallet' : 'hold' });
      else worker.postMessage({ type: 'cmd', action: kind });
    });
  }

  // 视频演示序列（对应参考样机的动作）
  for (const b of document.querySelectorAll('[data-demo]')) {
    b.addEventListener('click', () => {
      const name = b.dataset.demo;
      if (source === 'fw' && fw) {
        // 固件侧没有实现演示序列，切回 JS 仿真
        log('warn', '演示序列仅在 JS 仿真中可用，请切换数据源');
        return;
      }
      if (name === 'stop') {
        worker.postMessage({ type: 'cmd', action: 'hold' });
        log('info', '演示已停止');
      } else {
        worker.postMessage({ type: 'cmd', action: 'demo', arg: { name, loop: false } });
        log('info', `开始演示：${b.textContent}`);
      }
    });
  }

  $('load').addEventListener('input', (e) => {
    const kg = Number(e.target.value);
    $('loadVal').textContent = `${kg.toFixed(1)} kg`;
    if (source === 'fw' && fw) fw.send({ type: 'load', kg });
    else worker.postMessage({ type: 'cmd', action: 'setLoad', arg: { kg } });
  });

  $('alloc').addEventListener('change', (e) => {
    const mode = Number(e.target.value);
    if (source === 'fw' && fw) fw.send({ type: 'alloc', mode });
    else worker.postMessage({ type: 'cmd', action: 'setAlloc', arg: { mode } });
  });

  $('showFrame').addEventListener('change', (e) => { scene.show.frame = e.target.checked; scene.show.motors = e.target.checked; });
  $('showCables').addEventListener('change', (e) => { scene.show.cables = e.target.checked; });
  $('showTarget').addEventListener('change', (e) => { scene.show.target = e.target.checked; });
  $('showForces').addEventListener('change', (e) => { scene.show.forces = e.target.checked; });
  $('colorMode').addEventListener('change', (e) => { scene.colorMode = e.target.value; });

  for (const b of document.querySelectorAll('#viewBtns button')) {
    b.addEventListener('click', () => {
      const v = b.dataset.view;
      const views = {
        iso: { yaw: -0.7, pitch: 0.28, dist: 2.1 },
        front: { yaw: 0, pitch: 0.05, dist: 2.0 },
        side: { yaw: Math.PI / 2, pitch: 0.05, dist: 2.0 },
        top: { yaw: 0, pitch: 1.35, dist: 1.9 },
      };
      Object.assign(camera, views[v]);
      camera.target = [0, 0.62, 0];
    });
  }

  $('mode').addEventListener('change', (e) => switchSource(e.target.value));

  bindPlanUi();
  renderPathList();
}

// ---------------------------------------------------------------------------
// 数据源切换：JS 仿真 / STM32 固件无板仿真
// ---------------------------------------------------------------------------
/** 固件模式下：把固件输出的 (位姿, 8 根绳长, 8 根绳张力) 转成场景快照 */
let fwLast = null;
function fwSnapshot(msg) {
  const q = msg.q || [0, 0.62, 0, 0, 0, 0];
  // 固件回传的 trq 是每根绳的张力指令（N），不是电机力矩
  const T = (msg.trq || new Array(8).fill(0)).map((v) => Math.abs(v));
  const len = msg.len || new Array(8).fill(0);
  fwLast = {
    t: msg.t || 0,
    q: q.concat([0, 0, 0]).slice(0, 6),
    qTarget: q.concat([0, 0, 0]).slice(0, 6),
    T,
    Tref: T,
    l: len,
    acts: len.map((v, i) => ({
      cablePos: 0, cableVel: 0, motorVel: 0, motorTrq: msg.trq ? msg.trq[i] : 0,
      cur: 0, followingErr: 0, steps: 0, enabled: true, fault: 0,
    })),
    contact: false,
    feasible: !!msg.feasible,
    vp: [0, 0, 0],
    w: [0, 0, 0],
  };
  return fwLast;
}

async function switchSource(value) {
  source = value;
  if (value === 'fw') {
    log('info', '切换到固件模式：启动 STM32 控制代码的无板仿真（board_sim.exe）…');
    $('runState').textContent = '固件启动中…';
    try {
      const mod = await import('./firmware-source.js');
      fw = await mod.startFirmwareSim({
        onLog: (lvl, text) => log(lvl, text),
        onCmd: (msg) => {
          const snap = fwSnapshot(msg);
          if (source === 'fw') lastSnap = snap;
        },
      });
      // 固件模式以固定节拍推进：每帧请求 N 步（N 由倍率决定）
      startFwPump();
      $('runState').textContent = '运行中（STM32 固件 · 无板仿真）';
      $('runState').className = 'pill ok';
      log('info', '固件模式：控制周期 1 kHz，页面每帧推进若干步；绳索长度/张力由固件计算');
    } catch (e) {
      log('error', `固件模式启动失败：${e.message}。已回退到 JS 仿真。`);
      $('mode').value = 'js';
      source = 'js';
      $('runState').textContent = '运行中（JS 仿真）';
      $('runState').className = 'pill ok';
    }
  } else if (fw) {
    stopFwPump();
    fw.stop();
    fw = null;
    $('runState').textContent = '运行中（JS 仿真）';
    $('runState').className = 'pill ok';
    log('info', '已切回 JS 仿真');
  }
}

boot();
