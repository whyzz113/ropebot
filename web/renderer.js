/**
 * 三维渲染：Canvas 2D 软件渲染器（无 WebGL 依赖，结果确定、跨浏览器一致）。
 *
 * 原理：
 *   - 场景由三角形（{pos, nrm, col}）与线段组成，全部在「米」坐标系
 *   - 每帧：模型矩阵 → 视图 → 透视投影 → 背面剔除 → 按平均深度排序（画家算法）
 *     → 用 Canvas 2D 填充多边形，颜色按 Lambert 光照计算
 *   - 线段（绳索、地面网格、目标位姿轮廓）直接投影后描边
 *
 * 对于本项目的规模（约 1.3 万个三角形）在普通机器上可稳定 30~60 fps；
 * 相比 WebGL 的优点是没有任何驱动/实现差异，截图与用户所见完全一致。
 */

// ---------------------------------------------------------------------------
// 向量 / 矩阵（列主序 4x4）
// ---------------------------------------------------------------------------
export function identity4() {
  return new Float64Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);
}

export function mul4(a, b) {
  const o = new Float64Array(16);
  for (let c = 0; c < 4; c++) {
    for (let r = 0; r < 4; r++) {
      let s = 0;
      for (let k = 0; k < 4; k++) s += a[k * 4 + r] * b[c * 4 + k];
      o[c * 4 + r] = s;
    }
  }
  return o;
}

/** 位姿 [x,y,z,roll,pitch,yaw] -> 列主序 4x4（R = Rz*Ry*Rx） */
export function poseMatrix(q) {
  const [x, y, z, roll, pitch, yaw] = q;
  const cr = Math.cos(roll), sr = Math.sin(roll);
  const cp = Math.cos(pitch), sp = Math.sin(pitch);
  const cy = Math.cos(yaw), sy = Math.sin(yaw);
  const m = new Float64Array(16);
  m[0] = cy * cp; m[1] = sy * cp; m[2] = -sp; m[3] = 0;
  m[4] = cy * sp * sr - sy * cr; m[5] = sy * sp * sr + cy * cr; m[6] = cp * sr; m[7] = 0;
  m[8] = cy * sp * cr + sy * sr; m[9] = sy * sp * cr - cy * sr; m[10] = cp * cr; m[11] = 0;
  m[12] = x; m[13] = y; m[14] = z; m[15] = 1;
  return m;
}

export function applyMat(m, p) {
  return [
    m[0] * p[0] + m[4] * p[1] + m[8] * p[2] + m[12],
    m[1] * p[0] + m[5] * p[1] + m[9] * p[2] + m[13],
    m[2] * p[0] + m[6] * p[1] + m[10] * p[2] + m[14],
  ];
}

function unit(a) {
  const l = Math.hypot(a[0], a[1], a[2]) || 1;
  return [a[0] / l, a[1] / l, a[2] / l];
}

// ---------------------------------------------------------------------------
// 渲染器
// ---------------------------------------------------------------------------
export class Renderer {
  constructor(canvas) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.ctx.lineJoin = 'round';
    this.dpr = 1;
    this.stats = { meshes: 0, tris: 0 };
  }

  /** 视口同步（按设备像素比缩放，保证清晰）；布局未就绪时返回 false */
  resize() {
    // DPR 上限从 2 降到 1.5：软件光栅化是 CPU 填充，像素数随 dpr² 增长，
    // 1.5 已足够清晰，填充量比 2 少 ~44%，是低成本的帧率提升。
    const dpr = Math.min(window.devicePixelRatio || 1, 1.5);
    const cw = this.canvas.clientWidth;
    const ch = this.canvas.clientHeight;
    if (cw < 2 || ch < 2) return false;
    const w = Math.floor(cw * dpr);
    const h = Math.floor(ch * dpr);
    if (this.canvas.width !== w || this.canvas.height !== h) {
      this.canvas.width = w;
      this.canvas.height = h;
    }
    this.dpr = dpr;
    this.width = cw;
    this.height = ch;
    return true;
  }

  /**
   * 注册三角形网格。
   * 输入是「扁平顶点列表」：[{pos:[x,y,z], nrm:[..], col:[..]}, ...]，
   * 每连续 3 个顶点构成一个三角形（boxTris / cylinderTris 都是这个格式）。
   */
  createMesh(verts) {
    this.stats.meshes++;
    const tris = [];
    for (let i = 0; i + 2 < verts.length; i += 3) {
      const a = verts[i], b = verts[i + 1], c = verts[i + 2];
      tris.push({
        verts: [
          [a.pos[0], a.pos[1], a.pos[2]],
          [b.pos[0], b.pos[1], b.pos[2]],
          [c.pos[0], c.pos[1], c.pos[2]],
        ],
        nrm: a.nrm || [0, 1, 0],
        col: a.col || [0.7, 0.7, 0.7],
      });
    }
    this.stats.tris += tris.length;
    return { tris };
  }

  /** 线段（每帧更新）：positions / colors 扁平数组 */
  updateLines(key, positions, colors) {
    if (!this.lines) this.lines = new Map();
    this.lines.set(key, { pos: positions, col: colors });
  }

  begin(clear) {
    const ctx = this.ctx;
    const d = this.dpr;
    ctx.setTransform(d, 0, 0, d, 0, 0);
    ctx.fillStyle = `rgb(${Math.round(clear[0] * 255)},${Math.round(clear[1] * 255)},${Math.round(clear[2] * 255)})`;
    ctx.fillRect(0, 0, this.width, this.height);
    this._batch = [];
  }

  /**
   * 把一个网格加入本帧批次。
   * @param {object} mesh createMesh 结果
   * @param {Float64Array} model 模型矩阵
   * @param {object} camera {view, proj, eye}
   */
  drawMesh(mesh, model, camera) {
    if (!mesh || !mesh.tris || !mesh.tris.length) return;
    const arr = this._collectTriangles(mesh, model, camera);
    const batch = this._batch;
    for (let i = 0; i < arr.length; i++) batch.push(arr[i]);
  }

  /**
   * 把一个网格投影 + 背面剔除 + 光照成屏幕三角形（不画图，只返回 {z, sp, fill}）。
   * drawMesh 与 drawStatic 共用，避免重复投影逻辑。
   */
  _collectTriangles(mesh, model, camera) {
    const view = camera.view;
    const proj = camera.proj;
    const eye = camera.eye;
    const w = this.width, h = this.height;
    const cullBackface = this.cullBackface !== false;
    const out = [];
    for (const t of mesh.tris) {
      const wp = t.verts.map((v) => applyMat(model, v));
      const vp = wp.map((v) => applyMat(view, v));
      // 背面剔除：用视图空间的几何法线与视线方向
      const e1 = [vp[1][0] - vp[0][0], vp[1][1] - vp[0][1], vp[1][2] - vp[0][2]];
      const e2 = [vp[2][0] - vp[0][0], vp[2][1] - vp[0][1], vp[2][2] - vp[0][2]];
      const gn = [
        e1[1] * e2[2] - e1[2] * e2[1],
        e1[2] * e2[0] - e1[0] * e2[2],
        e1[0] * e2[1] - e1[1] * e2[0],
      ];
      const toEye = [-vp[0][0], -vp[0][1], -vp[0][2]];
      if (cullBackface && gn[0] * toEye[0] + gn[1] * toEye[1] + gn[2] * toEye[2] <= 0) continue;

      // 投影到屏幕（vp 是视图空间坐标，只用投影矩阵，不二次乘模型）
      let ok = true;
      const sp = [];
      let depth = 0;
      for (let k = 0; k < 3; k++) {
        const p = vp[k];
        const clip = [
          proj[0] * p[0] + proj[4] * p[1] + proj[8] * p[2] + proj[12],
          proj[1] * p[0] + proj[5] * p[1] + proj[9] * p[2] + proj[13],
          proj[2] * p[0] + proj[6] * p[1] + proj[10] * p[2] + proj[14],
          proj[3] * p[0] + proj[7] * p[1] + proj[11] * p[2] + proj[15],
        ];
        if (clip[3] <= 0.01) { ok = false; break; }
        const x = (clip[0] / clip[3]) * 0.5 + 0.5;
        const y = 0.5 - (clip[1] / clip[3]) * 0.5;
        sp.push([x * w, y * h]);
        depth += -p[2];
      }
      if (!ok) continue;
      // 屏幕外剔除
      if (Math.max(sp[0][0], sp[1][0], sp[2][0]) < 0 || Math.min(sp[0][0], sp[1][0], sp[2][0]) > w) continue;
      if (Math.max(sp[0][1], sp[1][1], sp[2][1]) < 0 || Math.min(sp[0][1], sp[1][1], sp[2][1]) > h) continue;

      // 光照（世界空间法线 + 世界空间光源）
      const wn = applyNormal(model, t.nrm || [0, 1, 0]);
      const L = unit([1.0 - wp[0][0], 2.2 - wp[0][1], 1.5 - wp[0][2]]);
      const V = unit([eye[0] - wp[0][0], eye[1] - wp[0][1], eye[2] - wp[0][2]]);
      const Hv = unit([L[0] + V[0], L[1] + V[1], L[2] + V[2]]);
      // 双面光照：法线朝向观察者时用原法线，否则翻转，避免背面变全黑
      const facing = wn[0] * V[0] + wn[1] * V[1] + wn[2] * V[2] >= 0 ? 1 : -1;
      const nx = wn[0] * facing, ny = wn[1] * facing, nz = wn[2] * facing;
      const diff = Math.max(0, nx * L[0] + ny * L[1] + nz * L[2]);
      const spec = Math.pow(Math.max(0, nx * Hv[0] + ny * Hv[1] + nz * Hv[2]), 48) * 0.3;
      const amb = 0.34 + 0.2 * Math.max(0, ny);
      const c = t.col || [0.7, 0.7, 0.7];
      const shade = (v) => Math.min(255, Math.round((v * (amb + 0.78 * diff) + spec) * 255));
      const fill = `rgb(${shade(c[0])},${shade(c[1])},${shade(c[2])})`;
      out.push({ z: depth / 3, sp, fill });
    }
    return out;
  }

  _paintTri(ctx, tr) {
    ctx.beginPath();
    ctx.moveTo(tr.sp[0][0], tr.sp[0][1]);
    ctx.lineTo(tr.sp[1][0], tr.sp[1][1]);
    ctx.lineTo(tr.sp[2][0], tr.sp[2][1]);
    ctx.closePath();
    ctx.fillStyle = tr.fill;
    ctx.fill();
    ctx.strokeStyle = tr.fill;
    ctx.lineWidth = 0.7;
    ctx.stroke();
  }

  /**
   * 把「在世界坐标里静止不动」的若干网格（框架 + 电机）一次性光栅化到离屏画布缓存，
   * 之后只要相机视角不变就直接整图贴回（drawImage），不再对每个三角形重新投影/排序/填充。
   *
   * 这是帧率优化的核心：本项目的瓶颈是纯 CPU 软件光栅化，静态几何（约 4k 三角形）
   * 每帧都被重算重画；而仿真运行、相机不转时这些都是浪费。缓存后，稳态下每帧只重新
   * 绘制会动的动平台（~60 三角形）和绳索线段。
   *
   * 缓存键 = 相机 view + proj + 画布像素尺寸；旋转或缩放时键变化 → 重建一次。
   */
  drawStatic(meshes, camera) {
    let valid = false;
    for (const m of meshes) { if (m && m.tris && m.tris.length) { valid = true; break; } }
    if (!valid) return;
    const key = this._camKey(camera);
    let off = this._staticCache;
    if (!off || this._staticKey !== key || off.width !== this.canvas.width || off.height !== this.canvas.height) {
      off = off || document.createElement('canvas');
      off.width = this.canvas.width;
      off.height = this.canvas.height;
      const octx = off.getContext('2d');
      octx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
      octx.clearRect(0, 0, this.width, this.height);
      const I = identity4();
      for (const mesh of meshes) {
        if (!mesh || !mesh.tris || !mesh.tris.length) continue;
        const arr = this._collectTriangles(mesh, I, camera);
        arr.sort((a, b) => b.z - a.z);
        for (let i = 0; i < arr.length; i++) this._paintTri(octx, arr[i]);
      }
      this._staticCache = off;
      this._staticKey = key;
    }
    // 贴回主画布（主画布已 setTransform(dpr)，目标矩形用 CSS 尺寸即可 1:1 映射）
    this.ctx.drawImage(off, 0, 0, off.width, off.height, 0, 0, this.width, this.height);
  }

  _camKey(camera) {
    return camera.view.join(',') + '#' + camera.proj.join(',') + '#' + this.canvas.width + 'x' + this.canvas.height;
  }

  /** 把所有网格按深度排序后绘制（画家算法） */
  flush() {
    const ctx = this.ctx;
    const batch = this._batch;
    batch.sort((a, b) => b.z - a.z);
    for (const t of batch) {
      ctx.beginPath();
      ctx.moveTo(t.sp[0][0], t.sp[0][1]);
      ctx.lineTo(t.sp[1][0], t.sp[1][1]);
      ctx.lineTo(t.sp[2][0], t.sp[2][1]);
      ctx.closePath();
      ctx.fillStyle = t.fill;
      ctx.fill();
      // 细描边消除相邻三角形之间的缝
      ctx.strokeStyle = t.fill;
      ctx.lineWidth = 0.7;
      ctx.stroke();
    }
  }

  /** 绘制线段 */
  drawLines(key, camera, width = 1.5) {
    const e = this.lines && this.lines.get(key);
    if (!e || !e.pos.length) return;
    const ctx = this.ctx;
    const view = camera.view;
    const proj = camera.proj;
    const mvp = mul4(proj, view);
    const w = this.width, h = this.height;
    ctx.lineWidth = width;
    const n = e.pos.length / 3;
    for (let i = 0; i + 1 < n; i += 2) {
      const p0 = [e.pos[i * 3], e.pos[i * 3 + 1], e.pos[i * 3 + 2]];
      const p1 = [e.pos[(i + 1) * 3], e.pos[(i + 1) * 3 + 1], e.pos[(i + 1) * 3 + 2]];
      const c0 = projectPoint(mvp, p0, w, h);
      const c1 = projectPoint(mvp, p1, w, h);
      if (!c0 || !c1) continue;
      const col = [e.col[i * 3], e.col[i * 3 + 1], e.col[i * 3 + 2]];
      ctx.strokeStyle = `rgb(${Math.round(col[0] * 255)},${Math.round(col[1] * 255)},${Math.round(col[2] * 255)})`;
      ctx.beginPath();
      ctx.moveTo(c0[0], c0[1]);
      ctx.lineTo(c1[0], c1[1]);
      ctx.stroke();
    }
  }
}

function applyNormal(m, n) {
  return unit([
    m[0] * n[0] + m[4] * n[1] + m[8] * n[2],
    m[1] * n[0] + m[5] * n[1] + m[9] * n[2],
    m[2] * n[0] + m[6] * n[1] + m[10] * n[2],
  ]);
}

function projectPoint(mvp, p, w, h) {
  const clip = [
    mvp[0] * p[0] + mvp[4] * p[1] + mvp[8] * p[2] + mvp[12],
    mvp[1] * p[0] + mvp[5] * p[1] + mvp[9] * p[2] + mvp[13],
    mvp[2] * p[0] + mvp[6] * p[1] + mvp[10] * p[2] + mvp[14],
    mvp[3] * p[0] + mvp[7] * p[1] + mvp[11] * p[2] + mvp[15],
  ];
  if (clip[3] <= 0.01) return null;
  return [((clip[0] / clip[3]) * 0.5 + 0.5) * w, (0.5 - (clip[1] / clip[3]) * 0.5) * h];
}

// ---------------------------------------------------------------------------
// 几何体生成
// ---------------------------------------------------------------------------
/** 轴对齐盒子（中心在原点）：6 个面各自带法线，三角形逆时针朝外 */
export function boxTris(sx, sy, sz, color) {
  const hx = sx / 2, hy = sy / 2, hz = sz / 2;
  const faces = [
    { n: [1, 0, 0], v: [[hx, -hy, -hz], [hx, -hy, hz], [hx, hy, hz], [hx, hy, -hz]] },
    { n: [-1, 0, 0], v: [[-hx, -hy, hz], [-hx, -hy, -hz], [-hx, hy, -hz], [-hx, hy, hz]] },
    { n: [0, 1, 0], v: [[-hx, hy, -hz], [hx, hy, -hz], [hx, hy, hz], [-hx, hy, hz]] },
    { n: [0, -1, 0], v: [[-hx, -hy, hz], [hx, -hy, hz], [hx, -hy, -hz], [-hx, -hy, -hz]] },
    { n: [0, 0, 1], v: [[-hx, -hy, hz], [-hx, hy, hz], [hx, hy, hz], [hx, -hy, hz]] },
    { n: [0, 0, -1], v: [[hx, -hy, -hz], [hx, hy, -hz], [-hx, hy, -hz], [-hx, -hy, -hz]] },
  ];
  const tris = [];
  for (const f of faces) {
    const [a, b, c, d] = f.v;
    // 顶点顺序按「从外部看逆时针」排列，使几何法线与 f.n 同向
    tris.push({ pos: a, nrm: f.n, col: color }, { pos: c, nrm: f.n, col: color }, { pos: b, nrm: f.n, col: color });
    tris.push({ pos: a, nrm: f.n, col: color }, { pos: d, nrm: f.n, col: color }, { pos: c, nrm: f.n, col: color });
  }
  return tris;
}

/** 圆柱（轴沿 +y，中心在原点） */
export function cylinderTris(r, h, color, seg = 14) {
  const tris = [];
  const hy = h / 2;
  for (let i = 0; i < seg; i++) {
    const a0 = (i / seg) * Math.PI * 2;
    const a1 = ((i + 1) / seg) * Math.PI * 2;
    const c0 = Math.cos(a0), s0 = Math.sin(a0), c1 = Math.cos(a1), s1 = Math.sin(a1);
    const p00 = [c0 * r, -hy, s0 * r], p01 = [c0 * r, hy, s0 * r];
    const p10 = [c1 * r, -hy, s1 * r], p11 = [c1 * r, hy, s1 * r];
    const n0 = [c0, 0, s0], n1 = [c1, 0, s1];
    // 侧面两个三角形（逆时针朝外）
    tris.push({ pos: p00, nrm: n0, col: color }, { pos: p11, nrm: n1, col: color }, { pos: p10, nrm: n1, col: color });
    tris.push({ pos: p00, nrm: n0, col: color }, { pos: p01, nrm: n0, col: color }, { pos: p11, nrm: n1, col: color });
    tris.push({ pos: [0, hy, 0], nrm: [0, 1, 0], col: color }, { pos: p11, nrm: [0, 1, 0], col: color }, { pos: p01, nrm: [0, 1, 0], col: color });
    tris.push({ pos: [0, -hy, 0], nrm: [0, -1, 0], col: color }, { pos: p00, nrm: [0, -1, 0], col: color }, { pos: p10, nrm: [0, -1, 0], col: color });
  }
  return tris;
}

/** 平移 + 缩放 */
export function transformed(tris, translate, scale = [1, 1, 1]) {
  const [tx, ty, tz] = translate;
  const [sx, sy, sz] = scale;
  return tris.map((t) => ({
    pos: [t.pos[0] * sx + tx, t.pos[1] * sy + ty, t.pos[2] * sz + tz],
    nrm: unit([t.nrm[0] / Math.abs(sx), t.nrm[1] / Math.abs(sy), t.nrm[2] / Math.abs(sz)]),
    col: t.col,
  }));
}

/** 绕 y 轴旋转 angle 再平移（把 y 轴圆柱摆成 x 轴） */
export function rotateYThenTranslate(tris, angle, translate) {
  const c = Math.cos(angle), s = Math.sin(angle);
  const [tx, ty, tz] = translate;
  return tris.map((t) => ({
    pos: [t.pos[0] * c + t.pos[2] * s + tx, t.pos[1] + ty, -t.pos[0] * s + t.pos[2] * c + tz],
    nrm: unit([t.nrm[0] * c + t.nrm[2] * s, t.nrm[1], -t.nrm[0] * s + t.nrm[2] * c]),
    col: t.col,
  }));
}

/** 两点之间的细杆（用于盘条、连杆） */
export function rodTris(a, b, r, color, seg = 10) {
  const d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
  const len = Math.hypot(d[0], d[1], d[2]) || 1e-9;
  const y = [d[0] / len, d[1] / len, d[2] / len];
  const ref = Math.abs(y[1]) < 0.9 ? [0, 1, 0] : [0, 0, 1];
  const x = unit([y[1] * ref[2] - y[2] * ref[1], y[2] * ref[0] - y[0] * ref[2], y[0] * ref[1] - y[1] * ref[0]]);
  const z = [y[1] * x[2] - y[2] * x[1], y[2] * x[0] - y[0] * x[2], y[0] * x[1] - y[1] * x[0]];
  const base = cylinderTris(r, len, color, seg);
  const mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2];
  return base.map((t) => ({
    pos: [
      t.pos[0] * x[0] + t.pos[1] * y[0] + t.pos[2] * z[0] + mid[0],
      t.pos[0] * x[1] + t.pos[1] * y[1] + t.pos[2] * z[1] + mid[1],
      t.pos[0] * x[2] + t.pos[1] * y[2] + t.pos[2] * z[2] + mid[2],
    ],
    nrm: unit([
      t.nrm[0] * x[0] + t.nrm[1] * y[0] + t.nrm[2] * z[0],
      t.nrm[0] * x[1] + t.nrm[1] * y[1] + t.nrm[2] * z[1],
      t.nrm[0] * x[2] + t.nrm[1] * y[2] + t.nrm[2] * z[2],
    ]),
    col: t.col,
  }));
}

// ---------------------------------------------------------------------------
// 相机
// ---------------------------------------------------------------------------
/** 轨道相机：左键旋转、Shift+左键/右键平移、滚轮缩放 */
export class OrbitCamera {
  constructor(canvas) {
    this.canvas = canvas;
    // 默认视角：对准平台工作高度，距离覆盖整个 1.2 m 框架
    this.target = [0, 0.62, 0];
    this.dist = 2.35;
    this.yaw = -0.75;
    this.pitch = 0.16;
    this.eye = [0, 0, 0];
    this.view = new Float64Array(16);
    this.proj = new Float64Array(16);
    let drag = null;
    canvas.addEventListener('contextmenu', (e) => e.preventDefault());
    canvas.addEventListener('mousedown', (e) => {
      drag = { x: e.clientX, y: e.clientY, btn: e.button, shift: e.shiftKey };
    });
    window.addEventListener('mouseup', () => { drag = null; });
    window.addEventListener('mousemove', (e) => {
      if (!drag) return;
      const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
      drag.x = e.clientX; drag.y = e.clientY;
      if (drag.btn === 0 && !drag.shift) {
        this.yaw -= dx * 0.007;
        this.pitch = Math.max(-1.45, Math.min(1.45, this.pitch + dy * 0.007));
      } else {
        const s = this.dist * 0.0018;
        this.target[0] -= Math.cos(this.yaw) * dx * s;
        this.target[2] += Math.sin(this.yaw) * dx * s;
        this.target[1] += dy * s;
      }
    });
    canvas.addEventListener('wheel', (e) => {
      e.preventDefault();
      this.dist = Math.max(0.4, Math.min(6, this.dist * (1 + Math.sign(e.deltaY) * 0.08)));
    }, { passive: false });
  }

  update(aspect) {
    const cp = Math.cos(this.pitch);
    this.eye = [
      this.target[0] + this.dist * cp * Math.sin(this.yaw),
      this.target[1] + this.dist * Math.sin(this.pitch),
      this.target[2] + this.dist * cp * Math.cos(this.yaw),
    ];
    this.view = lookAt(this.eye, this.target, [0, 1, 0]);
    this.proj = perspective((42 * Math.PI) / 180, aspect, 0.02, 30);
  }
}

function lookAt(eye, target, up) {
  const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
  const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
  const cr = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  const z = unit(sub(eye, target));
  const x = unit(cr(up, z));
  const y = cr(z, x);
  const m = new Float64Array(16);
  m[0] = x[0]; m[4] = x[1]; m[8] = x[2]; m[12] = -dot(x, eye);
  m[1] = y[0]; m[5] = y[1]; m[9] = y[2]; m[13] = -dot(y, eye);
  m[2] = z[0]; m[6] = z[1]; m[10] = z[2]; m[14] = -dot(z, eye);
  m[3] = 0; m[7] = 0; m[11] = 0; m[15] = 1;
  return m;
}

function perspective(fovy, aspect, near, far) {
  const f = 1 / Math.tan(fovy / 2);
  const m = new Float64Array(16);
  m[0] = f / aspect;
  m[5] = f;
  m[10] = (far + near) / (near - far);
  m[11] = -1;
  m[14] = (2 * far * near) / (near - far);
  m[15] = 0;
  return m;
}
