/**
 * 绳驱并联机器人共享核心（JavaScript 版）
 *
 * 与 firmware/core/*.c 逐项对应（test/compare_mechanics.mjs 校验一致性）。
 * 单位：m、kg、s、N、rad。
 *
 * 构型：8 绳冗余 6 自由度（4 组「一上一下」拉对），与框架的四层夹层、
 * 每层两台带卷筒的 86 闭环电机一一对应。
 *
 * ── 符号约定（与 C 版一致，务必遵守）───────────────────────────────
 *   u_i = A_i - B_i ：由平台系绳点指向卷筒的向量，l_i = |u_i|，s_i = u_i / l_i
 *   绳对平台的拉力 F_i = T_i * s_i （沿锚点方向，锚点高于平台时向上）
 *   w 定义为「平台作用于绳的等效力」，w = A(q) T，A_i = [s_i ; d_i × s_i]
 *   d_i = B_i - p。锚点整体高于平台时，锁定平衡要求 w = +m g e_y。
 * ────────────────────────────────────────────────────────────────
 */

export const PARAMS_URL = new URL('./cdpr_params.json', import.meta.url);
export const PI = Math.PI;
export const DEG = PI / 180.0;

// ---------------------------------------------------------------------------
// 三维向量
// ---------------------------------------------------------------------------
export const v3 = {
  sub: (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]],
  add: (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]],
  mul: (a, s) => [a[0] * s, a[1] * s, a[2] * s],
  dot: (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2],
  cross: (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]],
  norm: (a) => Math.hypot(a[0], a[1], a[2]),
};

/** ZYX 欧拉角：R = Rz(yaw) * Ry(pitch) * Rx(roll) */
export function rpyToR(rpy) {
  const [r, p, y] = rpy;
  const cr = Math.cos(r), sr = Math.sin(r);
  const cp = Math.cos(p), sp = Math.sin(p);
  const cy = Math.cos(y), sy = Math.sin(y);
  return [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
    [-sp, cp * sr, cp * cr],
  ];
}

export function worldToRpyRate(rpy, w) {
  const [r, p] = rpy;
  const cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p);
  const cpi = 1 / Math.max(Math.abs(cp), 1e-3);
  return [w[0] + sr * sp * cpi * w[1] + cr * sp * cpi * w[2], cr * w[1] - sr * w[2], sr * cpi * w[1] + cr * cpi * w[2]];
}

// ---------------------------------------------------------------------------
// 矩阵（行优先稠密，尺寸显式；所有运算都带维度校验）
// ---------------------------------------------------------------------------
export function matNew(rows, cols) {
  if (!(rows > 0 && cols > 0)) throw new Error(`matNew 尺寸非法 ${rows}x${cols}`);
  return { rows, cols, d: new Float64Array(rows * cols) };
}

export function matGet(M, i, j) {
  return M.d[i * M.cols + j];
}

export function matSet(M, i, j, v) {
  M.d[i * M.cols + j] = v;
}

export function matMulVec(M, x) {
  if (x.length !== M.cols) throw new Error(`matMulVec 维度不匹配 ${M.rows}x${M.cols} * ${x.length}`);
  const y = new Float64Array(M.rows);
  for (let i = 0; i < M.rows; i++) {
    let s = 0;
    const o = i * M.cols;
    for (let j = 0; j < M.cols; j++) s += M.d[o + j] * x[j];
    y[i] = s;
  }
  return y;
}

export function matT(M) {
  const T = matNew(M.cols, M.rows); // 步长是 M.rows
  for (let i = 0; i < M.rows; i++)
    for (let j = 0; j < M.cols; j++) T.d[i + j * M.rows] = M.d[j + i * M.cols];
  return T;
}

export function matMul(A, B) {
  if (A.cols !== B.rows) throw new Error(`matMul 维度不匹配 ${A.rows}x${A.cols} * ${B.rows}x${B.cols}`);
  const C = matNew(A.rows, B.cols);
  for (let i = 0; i < A.rows; i++) {
    for (let k = 0; k < A.cols; k++) {
      const a = A.d[i * A.cols + k];
      if (a === 0) continue;
      const cRow = i * B.cols;
      const bRow = k * B.cols;
      for (let j = 0; j < B.cols; j++) C.d[cRow + j] += a * B.d[bRow + j];
    }
  }
  return C;
}

/** 启动自检：矩阵乘法 / 转置 */
export function matrixSelfTest() {
  const A = matNew(2, 3);
  A.d.set([1, 2, 3, 4, 5, 6]);
  const B = matNew(3, 2);
  B.d.set([7, 8, 9, 10, 11, 12]);
  const C = matMul(A, B);
  const At = matT(A);
  const okMul = Math.abs(C.d[0] - 58) < 1e-9 && Math.abs(C.d[1] - 64) < 1e-9
    && Math.abs(C.d[2] - 139) < 1e-9 && Math.abs(C.d[3] - 154) < 1e-9;
  const okT = At.rows === 3 && At.cols === 2 && At.d[0] === 1 && At.d[1] === 4
    && At.d[2] === 2 && At.d[3] === 5 && At.d[4] === 3 && At.d[5] === 6;
  return { ok: okMul && okT, okMul, okT };
}

export { minNormSolve, solveDense } from './solve.mjs';
import { minNormSolve } from './solve.mjs';

// ---------------------------------------------------------------------------
// 参数
// ---------------------------------------------------------------------------
export function mergeParams(base, over) {
  if (over === undefined) return base;
  if (over === null || typeof over !== 'object') return over;
  if (Array.isArray(over)) return over.slice();
  const out = base && typeof base === 'object' && !Array.isArray(base) ? { ...base } : {};
  for (const k of Object.keys(over)) {
    if (k.startsWith('$')) continue;
    out[k] = mergeParams(base ? base[k] : undefined, over[k]);
  }
  return out;
}

export function validateParams(p) {
  const e = [];
  const num = (v, name, lo, hi) => {
    if (typeof v !== 'number' || !isFinite(v)) e.push(`${name} 必须是数值`);
    else if (lo !== undefined && v < lo) e.push(`${name} 必须 >= ${lo}（当前 ${v}）`);
    else if (hi !== undefined && v > hi) e.push(`${name} 必须 <= ${hi}（当前 ${v}）`);
  };
  num(p.frames?.width, 'frames.width', 0.2, 5);
  num(p.frames?.top_y, 'frames.top_y', 0.2, 5);
  num(p.winders?.half_spacing_x, 'winders.half_spacing_x', 0.02, 2);
  num(p.winders?.half_spacing_z, 'winders.half_spacing_z', 0.02, 2);
  num(p.winders?.drum_radius, 'winders.drum_radius', 0.003, 0.2);
  num(p.winders?.upper_y, 'winders.upper_y', 0.1, 5);
  num(p.winders?.lower_y, 'winders.lower_y', 0.05, 5);
  num(p.platform?.mass, 'platform.mass', 0.01, 500);
  num(p.platform?.half_size_x, 'platform.half_size_x', 0.005, 1);
  num(p.platform?.half_size_z, 'platform.half_size_z', 0.005, 1);
  num(p.cables?.t_min, 'cables.t_min', 0, 5000);
  num(p.cables?.t_max, 'cables.t_max', 0, 20000);
  num(p.cables?.emodulus, 'cables.emodulus', 1e3, 1e13);
  num(p.cables?.diameter, 'cables.diameter', 1e-5, 0.05);
  num(p.actuators?.hold_torque, 'actuators.hold_torque', 0.01, 200);
  num(p.actuators?.gear_ratio, 'actuators.gear_ratio', 0.1, 200);
  num(p.actuators?.driver_microstep, 'actuators.driver_microstep', 1, 512);
  num(p.actuators?.encoder_counts_per_rev, 'actuators.encoder_counts_per_rev', 100, 1e6);
  num(p.actuators?.step_angle_deg, 'actuators.step_angle_deg', 0.1, 30);
  if (p.cables && p.cables.t_min >= p.cables.t_max) e.push('cables.t_min 必须小于 cables.t_max');
  if (p.winders && p.winders.lower_y >= p.winders.upper_y) e.push('winders.lower_y 必须小于 winders.upper_y');
  const hp = p.controller?.home_pose;
  if (!Array.isArray(hp) || hp.length !== 6) e.push('controller.home_pose 需要 6 个分量');
  if (Array.isArray(hp) && p.winders) {
    const upY = p.winders.upper_y, loY = p.winders.lower_y;
    if (p.winders.topology === 'corner') {
      // 严格 8 角：锚点在框架 4 个角上（上下各一台），
      // 平台必须落在「下锚点之上、上锚点之下」才能形成双向力封闭。
      if (!(hp[1] > loY + 0.02 && hp[1] < upY - 0.02)) {
        e.push(`controller.home_pose 的高度必须落在下/上锚点之间（${loY.toFixed(2)} < y < ${upY.toFixed(2)}）`);
      }
    } else if (!(hp[1] < Math.min(upY, loY) - 0.02)) {
      // 边中点拓扑：上下两组锚点都在平台上方，靠「上组向上拉 / 下组向下拉」形成力封闭
      e.push('controller.home_pose 的高度必须低于下卷筒（否则绳子无法向下拉平台）');
    }
  }
  return e;
}

// ---------------------------------------------------------------------------
// 拓扑
// ---------------------------------------------------------------------------
/**
 * 锚点索引 i = 2j + k（j=0..3 平面，k=0 上卷筒 / 1 下卷筒）。
 * 平面顺序：j0(+x,+z) j1(-x,+z) j2(-x,-z) j3(+x,-z)。
 * 平台系绳点与锚点一一对应：k=0 取上表面，k=1 取下表面。
 */
export function buildModel(p) {
  const w = p.winders;
  const anchors = [];
  const hx0 = p.platform.half_size_x, hz0 = p.platform.half_size_z;
  const platY = p.controller.home_pose[1];
  const t0 = p.platform.thickness;

  if (w.topology === 'corner') {
    // ------------------------------------------------------------------
    // 严格 8 角拓扑（按用户模型）：
    //   8 台电机分居框架 8 个角（4 个水平角 × 上/下两层）
    //   8 个系绳点分居动平台 8 个角（4 个水平角 × 上/下表面）
    //   每个角的上下两台电机，各接该角对应的上/下系绳点。
    //
    // i = 2j + k（j 为水平角：+x+z, +x-z, -x-z, -x+z；k=0 上绳 / 1 下绳）
    //   锚点：第 j 个角的框架位置（±corner_spacing），上绳 upper_y、下绳 lower_y
    //   系绳点：第 j 个角的平台位置（±half_size），上绳上表面、下绳下表面
    //
    // ⚠️ 该构型存在几何退化（详见 README §5）：
    //   - 平台为正方形且角点与锚点径向对齐时，A 的 My 行恒为 0（偏航不可控）
    //   - 上下锚点在同一条竖棱上，零空间缺少全正向量（部分绳无法绷紧）
    //   本实现如实还原该模型；若需要 6 自由度全可控，请用 topology: 'edge'。
    // ------------------------------------------------------------------
    const S = w.corner_spacing ?? 0.485;
    const UP = w.upper_y, LO = w.lower_y;
    const corners = [[1, 1], [1, -1], [-1, -1], [-1, 1]];
    const pts = [];
    for (let j = 0; j < 4; j++) {
      const [sx, sz] = corners[j];
      // 上绳：该角的顶部锚点
      anchors.push([sx * S, UP, sz * S]);
      // 下绳：同一角的底部锚点
      anchors.push([sx * S, LO, sz * S]);
      // 上绳接平台该角的上表面，下绳接该角的下表面
      pts.push([sx * hx0, t0 / 2, sz * hz0]);
      pts.push([sx * hx0, -t0 / 2, sz * hz0]);
    }
    const planes = corners.map(([sx, sz]) => [sx * S, sz * S]);
    return {
      params: p,
      nCables: 8,
      nPairs: 4,
      anchors,
      platformPointsLocal: pts,
      pairOfCable: [0, 0, 1, 1, 2, 2, 3, 3],
      planes,
      topology: 'corner',
    };
  }

  if (Array.isArray(w.deck_levels) && w.deck_levels.length === 4) {
    // ------------------------------------------------------------------
    // 「新框架」装配体拓扑：4 层甲板，每层 2 台电机（分居相对两边，
    // 逐层旋转 90°），共 8 个卷筒/锚点，层高逐层递减。
    // 平台有 8 个系绳点：4 个角 × (上表面 / 下表面)。
    // 每根绳连到「离它最近的角点」，因此三维中绳索不会交叉。
    // ------------------------------------------------------------------
    const dirs = [[1, 0], [0, 1], [-1, 0], [0, -1]];
    const S = w.half_spacing_x;
    const spread = w.deck_spread ?? 0.08;
    const anchorsRaw = [];
    for (let L = 0; L < 4; L++) {
      const [nx, nz] = dirs[L];
      const tu = nx !== 0 ? [0, 1] : [1, 0];
      for (const s of [-1, 1]) {
        anchorsRaw.push([
          nx * S + tu[0] * s * spread,
          w.deck_levels[L],
          nz * S + tu[1] * s * spread,
        ]);
      }
    }
    // 8 个候选系绳点：4 角 × 上下表面
    const corners = [[1, 1], [-1, 1], [-1, -1], [1, -1]];
    const cand = [];
    for (const [sx, sz] of corners) {
      cand.push([sx * hx0, t0 / 2, sz * hz0]);
      cand.push([sx * hx0, -t0 / 2, sz * hz0]);
    }
    // 每根绳选最近的候选点（保证画面不交叉）
    const pts = anchorsRaw.map((a) => {
      let best = cand[0];
      let bd = Infinity;
      for (const q of cand) {
        const d = Math.hypot(a[0] - q[0], (platY + q[1]) - a[1], a[2] - q[2]);
        if (d < bd) { bd = d; best = q; }
      }
      return [best[0], best[1], best[2]];
    });
    const planes = dirs.map(([nx, nz]) => [nx * S, nz * S]);
    return {
      params: p,
      nCables: 8,
      nPairs: 4,
      anchors: anchorsRaw,
      platformPointsLocal: pts,
      pairOfCable: [0, 0, 1, 1, 2, 2, 3, 3],
      planes,
      deckLevels: w.deck_levels.slice(),
    };
  }

  // --------------------------------------------------------------------
  // 标准拓扑：锚点在「四边中点」（法向 half_spacing），每边上下各一台电机
  // （沿边微错开 ±edge_offset，高度 upper_y / lower_y）。
  //
  // 系绳点：每边两个角点，上绳接该边一角的上表面、下绳接该边另一角的下表面。
  //   - 每根绳都连到「同边、同侧」的角点，俯视图上不跨到对角（不交叉）
  //   - 上下两绳的力臂方向不同，A 的力矩行满秩（偏航/横滚可控）
  //   - 平台位于 upper_y 与 lower_y 之间，形成双向力封闭
  // --------------------------------------------------------------------
  const S = w.half_spacing_x;
  const off = w.edge_offset ?? 0.04;
  const UP = w.upper_y, LO = w.lower_y;
  const sides = [[1, 0], [0, 1], [-1, 0], [0, -1]]; // +x, +z, -x, -z
  // 每条边的两个角点（顺序与 sides 对应）
  const edgeCorners = [
    [[1, 1], [1, -1]],    // +x 边：+z 角 / -z 角
    [[1, 1], [-1, 1]],    // +z 边：+x 角 / -x 角
    [[-1, 1], [-1, -1]],  // -x 边：+z 角 / -z 角
    [[1, -1], [-1, -1]],  // -z 边：+x 角 / -x 角
  ];
  const pts = [];
  for (let j = 0; j < 4; j++) {
    const [nx, nz] = sides[j];
    const tu = nx !== 0 ? [0, 1] : [1, 0];
    const [[ax, az], [bx, bz]] = edgeCorners[j];
    anchors.push([nx * S + tu[0] * off, UP, nz * S + tu[1] * off]);
    anchors.push([nx * S - tu[0] * off, LO, nz * S - tu[1] * off]);
    pts.push([ax * hx0, t0 / 2, az * hz0]);
    pts.push([bx * hx0, -t0 / 2, bz * hz0]);
  }
  const planes = sides.map(([nx, nz]) => [nx * S, nz * S]);
  return {
    params: p,
    nCables: 8,
    nPairs: 4,
    anchors,
    platformPointsLocal: pts,
    pairOfCable: [0, 0, 1, 1, 2, 2, 3, 3],
    planes,
  };
}

export function poseToX(p, rpy) {
  return new Float64Array([p[0], p[1], p[2], rpy[0], rpy[1], rpy[2]]);
}

export function makeGeo(n = 8) {
  return {
    A: matNew(6, n),
    l: new Float64Array(n),
    s: Array.from({ length: n }, () => [0, 0, 0]),
    u: Array.from({ length: n }, () => [0, 0, 0]),
    B: Array.from({ length: n }, () => [0, 0, 0]),
    R: rpyToR([0, 0, 0]),
  };
}

export function structure(model, x, out) {
  const g = out || makeGeo(model.nCables);
  const n = model.nCables;
  const R = rpyToR([x[3], x[4], x[5]]);
  g.R = R;
  for (let i = 0; i < n; i++) {
    const b = model.platformPointsLocal[i];
    const Bx = x[0] + R[0][0] * b[0] + R[0][1] * b[1] + R[0][2] * b[2];
    const By = x[1] + R[1][0] * b[0] + R[1][1] * b[1] + R[1][2] * b[2];
    const Bz = x[2] + R[2][0] * b[0] + R[2][1] * b[1] + R[2][2] * b[2];
    const B = g.B[i];
    B[0] = Bx; B[1] = By; B[2] = Bz;
    const a = model.anchors[i];
    const ux = a[0] - Bx, uy = a[1] - By, uz = a[2] - Bz;
    const l = Math.max(Math.hypot(ux, uy, uz), 1e-6);
    const sx = ux / l, sy = uy / l, sz = uz / l;
    const U = g.u[i]; U[0] = ux; U[1] = uy; U[2] = uz;
    const S = g.s[i]; S[0] = sx; S[1] = sy; S[2] = sz;
    g.l[i] = l;
    const dx = Bx - x[0], dy = By - x[1], dz = Bz - x[2];
    g.A.d[0 * n + i] = sx;
    g.A.d[1 * n + i] = sy;
    g.A.d[2 * n + i] = sz;
    g.A.d[3 * n + i] = dy * sz - dz * sy;
    g.A.d[4 * n + i] = dz * sx - dx * sz;
    g.A.d[5 * n + i] = dx * sy - dy * sx;
  }
  return g;
}

/** 正运动学：绳长 -> 位姿（Gauss-Newton，J = dl/dx = -A^T） */
export function forwardKinematics(model, l, x0, iters = 60, tol = 1e-10) {
  const n = model.nCables;
  const x = Float64Array.from(x0);
  const g = makeGeo(n);
  const J = matNew(n, 6);
  const e = new Float64Array(n);
  let best = Infinity;
  const xBest = Float64Array.from(x);
  for (let it = 0; it < iters; it++) {
    structure(model, x, g);
    let err = 0;
    for (let i = 0; i < n; i++) {
      e[i] = l[i] - g.l[i];
      err = Math.max(err, Math.abs(e[i]));
    }
    if (err < best) {
      best = err;
      xBest.set(x);
    }
    if (err < tol) break;
    for (let i = 0; i < n; i++)
      for (let k = 0; k < 6; k++) J.d[i * 6 + k] = -g.A.d[k * n + i];
    const dx = minNormSolve(J, e);
    let step = 1, big = 0;
    for (let k = 0; k < 6; k++) big = Math.max(big, Math.abs(dx[k]));
    if (big > 0.15) step = 0.15 / big;
    for (let k = 0; k < 6; k++) x[k] += step * dx[k];
  }
  return Float64Array.from(xBest);
}

/** 绳弹性刚度 k = E·A / l */
export function cableStiffness(p, l) {
  const area = PI * (p.cables.diameter / 2) ** 2;
  return (p.cables.emodulus * area) / Math.max(l, 0.02);
}

// ---------------------------------------------------------------------------
// 张力分配
// ---------------------------------------------------------------------------
/**
 * 给定 w（= A T，锁定平衡时 = 重力 + 外部接触力），求 8 根绳张力。
 *
 * mode 0：T = 0（自检）
 * mode 1：最小范数解 min||T|| s.t. A T = w（默认；确定性、可达时精确）
 * mode 2：最小范数解 + 盒约束 t_min..t_max 交替投影（保证下界）
 * mode 3：神经启发式对照：最小范数解 + 拉对内部对抗预紧（不改变 w）
 */
export function allocateT(model, x, wrench, opts = {}) {
  const p = model.params;
  const tmin = opts.t_min ?? p.cables.t_min;
  const tmax = opts.t_max ?? p.cables.t_max;
  const mode = opts.mode ?? 1;
  const n = model.nCables;
  const { A } = structure(model, x);
  let T;
  if (mode === 0) {
    T = new Float64Array(n);
  } else if (mode === 1) {
    T = minNormSolve(A, wrench);
  } else if (mode === 3) {
    T = minNormSolve(A, wrench);
    T = addInternalPreload(model, x, A, T, opts.pretension ?? Math.max(tmin, 0) * 0.5);
  } else {
    // 盒约束求解（物理正确做法）：非负最小二乘 min ||A T - w||² s.t. T >= 0。
    // 投影梯度迭代到收敛，残差通常 < 0.1 N（相对 40 N 量级重力可忽略）。
    //
    // ⚠️ 角滑轮拓扑（4 角 × 上下两层）下，NNLS 会收敛到
    //    「上组全撑、下组全 0」的角点解：此时 w 仍被精确平衡（残差 0），
    //    但下组 4 根绳松弛 —— 这是非负最小二乘的固有退化，不是 bug。
    //    因此先求 NNLS，若发现有绳张力低于 t_min，再沿零空间加预紧。
    T = nonNegativeSolve(A, wrench, n, opts.iters ?? 20000);
    T = addPreload(model, x, A, T, tmin);
    for (let i = 0; i < n; i++) if (T[i] > tmax) T[i] = tmax;
  }
  let res = 0;
  for (let k = 0; k < 6; k++) {
    let s = wrench[k];
    for (let i = 0; i < n; i++) s -= A.d[k * n + i] * T[i];
    res = Math.max(res, Math.abs(s));
  }
  let fmin = Infinity, fmax = -Infinity;
  for (let i = 0; i < n; i++) { fmin = Math.min(fmin, T[i]); fmax = Math.max(fmax, T[i]); }
  const fc = forceClosure(model, x, T, { t_min: tmin });
  // 判据：
  //   1) 每根绳都必须「真正受拉」—— 用 t_min 的一个小比例作阈值，
  //      而不是 fmin > 0：角点类拓扑下 NNLS 会给某几根恰好 0 的退化解，
  //      那种解数值上「有张力」，实际一松就塌。
  //   2) 残差相对量级可忽略（非负最小二乘是迭代解，残差 ~0.1 N 属正常）。
  //
  // 注意：不要把阈值设成 t_min 本身。t_min 是「报警/建议下界」，
  // 而不是「低于它就一定不可行」的硬约束 —— 边中点拓扑在 left/right 等
  // 位姿下最松的一根本来就只有 2~3 N，那样会把可行位姿误判为失败。
  const wScale = Math.max(1e-6, Math.hypot(...Array.from(wrench).slice(0, 3)));
  const tensionFloor = Math.max(1e-3, tmin * 0.25);
  const feasible = fmin > tensionFloor && res < Math.max(1e-3, wScale * 0.02);
  return { T, fmin, fmax, residual: res, fcMargin: fc.margin, feasible };
}

/**
 * 非负最小二乘（NNLS）：min ||A T - w||² s.t. T >= 0。
 * 投影梯度 + 精确步长（1/L，L 为 A^T A 的最大特征值上界）。
 * 用于盒约束张力分配的初值：直接从物理可行的非负解出发，
 * 避免「最小范数解为负、再截断」导致的不可行。
 */
export function nonNegativeSolve(A, w, n, iters = 600) {
  // L 的上界：||A^T A||_inf（行绝对值和的最大值）
  let L = 0;
  for (let i = 0; i < 6; i++) {
    let s = 0;
    for (let j = 0; j < n; j++) s += Math.abs(A.d[i * n + j]);
    L = Math.max(L, s);
  }
  const lam = 1 / Math.max(L * L, 1e-9);
  const T = new Float64Array(n).fill(1);
  for (let it = 0; it < iters; it++) {
    const r = new Float64Array(6);
    for (let k = 0; k < 6; k++) {
      let s = w[k];
      for (let i = 0; i < n; i++) s -= A.d[k * n + i] * T[i];
      r[k] = s;
    }
    let move = 0;
    for (let i = 0; i < n; i++) {
      let g = 0;
      for (let k = 0; k < 6; k++) g += A.d[k * n + i] * r[k];
      const v = T[i] + lam * g;
      const nv = v > 0 ? v : 0;
      move = Math.max(move, Math.abs(nv - T[i]));
      T[i] = nv;
    }
    if (move < 1e-9) break;
  }
  return T;
}

/**
 * 带下界的张力分配：
 *   min ||A T - w||² + mu ||T - tmin||²   s.t.  tmin <= T <= tmax
 * 用投影梯度求解（步长 1/L，L 为 A^T A 的最大行和上界）。
 * 与「最小范数解」相比，它天然满足「每根绳至少 tmin 的预紧」，
 * 这正是实际 CDPR 控制器要保证的物理条件。
 */
export function boundedTensionSolve(A, w, n, tmin, tmax, iters = 3000) {
  let L = 0;
  for (let i = 0; i < 6; i++) {
    let s = 0;
    for (let j = 0; j < n; j++) s += Math.abs(A.d[i * n + j]);
    L = Math.max(L, s);
  }
  const lam = 1 / Math.max(L * L, 1e-9);
  const mu = 0.02; // 预紧权重：过大会牺牲结构方程精度
  const T = new Float64Array(n).fill(tmin);
  for (let it = 0; it < iters; it++) {
    const r = new Float64Array(6);
    for (let k = 0; k < 6; k++) {
      let s = w[k];
      for (let i = 0; i < n; i++) s -= A.d[k * n + i] * T[i];
      r[k] = s;
    }
    let move = 0;
    for (let i = 0; i < n; i++) {
      let g = 0;
      for (let k = 0; k < 6; k++) g += A.d[k * n + i] * r[k];
      g -= mu * (T[i] - tmin);
      const v = T[i] + lam * g;
      const nv = Math.min(tmax, Math.max(tmin, v));
      move = Math.max(move, Math.abs(nv - T[i]));
      T[i] = nv;
    }
    if (move < 1e-7) break;
  }
  return T;
}

/** 拉对内部预紧：沿 z_j = e_2j - e_2j+1 方向的零空间分量（A z 的补偿项） */
export function addInternalPreload(model, x, A, T, delta) {
  const n = model.nCables;
  const out = Float64Array.from(T);
  for (let j = 0; j < model.nPairs; j++) {
    const z = new Float64Array(n);
    z[2 * j] = 1;
    z[2 * j + 1] = -1;
    const dT = minNormSolve(A, matMulVec(A, z));
    for (let i = 0; i < n; i++) out[i] += delta * (z[i] - dT[i]);
  }
  return out;
}

/**
 * 对称预紧：把「上组 / 下组」整体沿零空间反向拉紧，保证每根绳都 >= t_min。
 *
 * 角滑轮拓扑（4 角 × 上下两层）下结构矩阵的零空间恰好含
 *   z = [1,1,1,1, -c,-c,-c,-c]，c = (Σ上组 s_y) / (Σ下组 s_y)
 * 即「上组全部 +1、下组全部 -c」不改变 A·T，却能把两组绳都拉紧。
 * NNLS 会退化成只撑上组（下组为 0），这里沿 z 把两组同时抬到 t_min 以上。
 *
 * 若零空间没有这样的方向（例如四边中点拓扑），本函数原样返回 T。
 */
export function addSymmetricPreload(model, x, A, T, tmin) {
  const n = model.nCables;
  if (n < 2 || n % 2 !== 0) return T;
  const half = n / 2;
  // 先看是否真有绳低于下界；没有就不动（保持 NNLS 的最小范数性）
  let need = false;
  for (let i = 0; i < n; i++) if (T[i] < tmin) { need = true; break; }
  if (!need) return T;

  // 求零空间方向 z：上半组全 1、下半组全 -c，使 A z = 0
  // 只需要竖直行（k=1）配平即可；其余行由拓扑对称性自然为零。
  let sumUp = 0, sumDn = 0;
  for (let i = 0; i < half; i++) sumUp += A.d[1 * n + i];
  for (let i = half; i < n; i++) sumDn += A.d[1 * n + i];
  if (Math.abs(sumDn) < 1e-9 || Math.abs(sumUp) < 1e-9) return T;
  const c = -sumUp / sumDn;
  if (!(c > 0)) return T;      // 方向反了说明拓扑不是「上/下」两组，放弃

  // 检查 z 是否真的在零空间里（用完整 A 验证，避免误伤别的拓扑）
  let maxAz = 0;
  for (let k = 0; k < 6; k++) {
    let s = 0;
    for (let i = 0; i < half; i++) s += A.d[k * n + i];
    for (let i = half; i < n; i++) s -= c * A.d[k * n + i];
    maxAz = Math.max(maxAz, Math.abs(s));
  }
  if (maxAz > 1e-6) return T;  // 不是零空间方向，保持原解

  // 沿 z 抬升，使两组最小值都 >= tmin
  let minUp = Infinity, minDn = Infinity;
  for (let i = 0; i < half; i++) minUp = Math.min(minUp, T[i]);
  for (let i = half; i < n; i++) minDn = Math.min(minDn, T[i]);
  const alphaUp = tmin - minUp;          // 上组需要 +alpha（z 的上半组是 +1）
  const alphaDn = (tmin - minDn) / c;    // 下组需要 -alpha*c，所以 alpha = -(tmin-minDn)/c
  const alpha = Math.max(alphaUp, -alphaDn);
  if (!(alpha > 0)) return T;
  const out = Float64Array.from(T);
  for (let i = 0; i < half; i++) out[i] += alpha;
  for (let i = half; i < n; i++) out[i] -= alpha * c;
  return out;
}

/**
 * 通用预紧入口：把张力抬到 t_min 以上，同时不改变 A·T（所以残差仍为 0）。
 *
 * 两条路径，按可靠性排序：
 *   1. findPositiveNullspace：在零空间里找「全正方向」，对任意拓扑都适用；
 *   2. addSymmetricPreload：老的「上组全 +1 / 下组全 -c」特例，作为兜底。
 *
 * 若都找不到可用方向，原样返回（该位姿不可力封闭）。
 */
export function addPreload(model, x, A, T, tmin) {
  const n = model.nCables;
  let need = false;
  for (let i = 0; i < n; i++) if (T[i] < tmin) { need = true; break; }
  if (!need) return T;

  // 路径 1：通用零空间正方向
  const z = findPositiveNullspace(A, n, 4000);
  if (z) {
    // z 必须是严格正的，否则「加 alpha 抬升」会压垮负分量那几根绳。
    // findPositiveNullspace 已保证 min(z) > 1e-4，这里再核一次以防数值抖动。
    let minZ = Infinity;
    for (let i = 0; i < n; i++) minZ = Math.min(minZ, z[i]);
    if (minZ > 1e-6) {
      // 求 alpha 使 T + alpha*z 的所有分量 >= tmin
      let alpha = -Infinity;
      for (let i = 0; i < n; i++) alpha = Math.max(alpha, (tmin - T[i]) / z[i]);
      if (isFinite(alpha) && alpha > 0) {
        const out = Float64Array.from(T);
        for (let i = 0; i < n; i++) out[i] += alpha * z[i];
        // 抬升后必须仍然满足 A·T = w（z 在零空间里，理论上自动满足）
        let err = 0;
        for (let k = 0; k < 6; k++) {
          let s = 0;
          for (let i = 0; i < n; i++) s += A.d[k * n + i] * z[i];
          err = Math.max(err, Math.abs(s));
        }
        if (err < 1e-6) return out;
      }
    }
  }

  // 路径 2：老的对称预紧（上组 / 下组）
  return addSymmetricPreload(model, x, A, T, tmin);
}

/**
 * 通用零空间预紧：在 A 的零空间里找「尽可能正」的方向 z，
 * 再沿 z 把张力抬到 t_min 以上（不改变 A·T）。
 *
 * 与 addSymmetricPreload 的区别：不假设「上组/下组」这种固定结构，
 * 而是对任意拓扑都适用（8 角锚点、边中点锚点都行）。
 *
 * 做法：用 Gram-Schmidt 求 A 的行空间正交基，投影矩阵 P = I - Σ b b^T，
 * 然后对 z 做「最大化最小分量」的梯度上升（z 始终保持在零空间内）。
 * 若零空间里不存在全正方向（该位姿无法力封闭），返回 null。
 *
 * @returns {Float64Array|null} 归一化的预紧方向（仅当所有分量为正时返回）
 */
export function findPositiveNullspace(A, n, iters = 4000) {
  // A 的行空间正交基
  const basis = [];
  for (let k = 0; k < 6; k++) {
    const v = new Float64Array(n);
    for (let i = 0; i < n; i++) v[i] = A.d[k * n + i];
    for (const b of basis) {
      let d = 0;
      for (let i = 0; i < n; i++) d += b[i] * v[i];
      for (let i = 0; i < n; i++) v[i] -= d * b[i];
    }
    let nv = 0;
    for (let i = 0; i < n; i++) nv += v[i] * v[i];
    nv = Math.sqrt(nv);
    if (nv > 1e-9) {
      for (let i = 0; i < n; i++) v[i] /= nv;
      basis.push(v);
    }
  }
  const proj = (v) => {
    const w = Float64Array.from(v);
    for (const b of basis) {
      let d = 0;
      for (let i = 0; i < n; i++) d += b[i] * w[i];
      for (let i = 0; i < n; i++) w[i] -= d * b[i];
    }
    return w;
  };

  // 初值：全 1 投影到零空间
  const one = new Float64Array(n).fill(1 / Math.sqrt(n));
  let z = proj(one);
  let nz = 0;
  for (let i = 0; i < n; i++) nz += z[i] * z[i];
  nz = Math.sqrt(nz);
  if (nz < 1e-6) return null;
  for (let i = 0; i < n; i++) z[i] /= nz;

  let best = Infinity, bestZ = null;
  const evaluate = (v) => {
    let m = Infinity;
    for (let i = 0; i < n; i++) m = Math.min(m, v[i]);
    return m;
  };
  best = evaluate(z);
  bestZ = Float64Array.from(z);

  for (let it = 0; it < iters; it++) {
    // 找最小分量的下标
    let minI = 0;
    for (let i = 1; i < n; i++) if (z[i] < z[minI]) minI = i;
    // 朝「提升该分量」的方向走，再投回零空间
    const grad = new Float64Array(n);
    grad[minI] = 1;
    const g2 = proj(grad);
    const step = 0.08;
    let nn = 0;
    for (let i = 0; i < n; i++) {
      z[i] += step * g2[i];
      nn += z[i] * z[i];
    }
    nn = Math.sqrt(nn);
    if (nn < 1e-9) break;
    for (let i = 0; i < n; i++) z[i] /= nn;
    const m = evaluate(z);
    if (m > best) { best = m; bestZ = Float64Array.from(z); }
  }
  return best > 1e-4 ? bestZ : null;
}

/**
 * 力封闭检验（张力空间）：平台在 6 个方向 ± 被外力推动 eps*e_k 时，
 * 存在 dT 使 A(T+dT) = w + dW 且 T+dT 仍 >= t_min。
 * margin > 0 表示该位姿处于力封闭工作空间内（可保持全部绳绷紧）。
 */
export function forceClosure(model, x, T, opts = {}) {
  const tmin = opts.t_min ?? model.params.cables.t_min;
  const n = model.nCables;
  // 单位外力扰动（1 N / 1 N·m），结果回除后与扰动幅度无关
  const { A } = structure(model, x);
  let margin = Infinity;
  let worst = { dir: -1, sign: 0, value: Infinity };
  for (let k = 0; k < 6; k++) {
    for (const sgn of [1, -1]) {
      const d = new Float64Array(6);
      d[k] = sgn;
      const dT = minNormSolve(A, d);
      for (let i = 0; i < n; i++) {
        const m = T[i] + dT[i] - tmin;
        if (m < margin) { margin = m; worst = { dir: k, sign: sgn, value: m }; }
      }
    }
  }
  return { margin, worst };
}

/**
 * 保持锁定所需的 w = A T。
 * 锚点高于平台时即 +m g e_y；若平台受到外部接触力 F_c（地面支撑、外部推力），
 * 则 w = +m g e_y - F_c，力矩部分同理取反。
 */
export function externalWrench(p, contactF = [0, 0, 0], contactM = [0, 0, 0]) {
  const W = new Float64Array(6);
  const m = p.platform.mass, g = p.platform.gravity;
  W[0] = -contactF[0];
  W[1] = m * g - contactF[1];
  W[2] = -contactF[2];
  W[3] = -contactM[0];
  W[4] = -contactM[1];
  W[5] = -contactM[2];
  return W;
}

/** 平台刚度 K = J^T diag(k_i) J（绳弹性），J = dl/dx = -A^T */
export function stiffnessMatrix(model, x) {
  const n = model.nCables;
  const g = structure(model, x);
  const K = matNew(6, 6);
  for (let i = 0; i < n; i++) {
    const k = cableStiffness(model.params, g.l[i]);
    for (let r = 0; r < 6; r++)
      for (let c = 0; c < 6; c++) K.d[r * 6 + c] += g.A.d[r * n + i] * k * g.A.d[c * n + i];
  }
  return K;
}
