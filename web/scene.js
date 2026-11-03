/**
 * 场景：按「新框架/新装配体.SLDASM」实测几何搭建（SolidWorks 2022 实测，2026-10-26）。
 *
 *   外框 1.03 × 1.03 × 1.33 m，30×30 铝型材 44 根（立柱、上下框、X 型斜撑）
 *   两块 988×988×10 mm 夹层板（y = 0.015 / 0.985）
 *   8 套电机（86 闭环电机 + 减速器 + 卷筒）：下组 4 台 y≈0.072、上组 4 台 y≈0.928
 *   8 个角滑轮（4 角 × 上下两层，R=34 mm，水平位于 ±0.431）
 *   动平台：上平台板 + 下层板 + 8 个系绳点
 *
 * 外观尺寸全部来自 cdpr_params.json 的 frames.members（SolidWorks 实测包围盒）；
 * 绳的拓扑仍由 winders / platform 决定，与 sim/core 的 buildModel 严格一致。
 */
import {
  boxTris, cylinderTris, transformed, rotateYThenTranslate,
  OrbitCamera, poseMatrix, applyMat, mul4,
} from './renderer.js';

const C = {
  metal: [0.58, 0.62, 0.67],
  metalDark: [0.30, 0.33, 0.37],
  plate: [0.45, 0.49, 0.54],
  deck: [0.38, 0.42, 0.48],
  motor: [0.18, 0.19, 0.22],
  motorCap: [0.30, 0.32, 0.36],
  gear: [0.75, 0.72, 0.65],
  drum: [0.82, 0.63, 0.25],
  platform: [0.95, 0.55, 0.15],
  platform2: [0.80, 0.83, 0.88],
  pulley: [0.86, 0.24, 0.22],
  bracket: [0.44, 0.48, 0.54],
  tie: [0.32, 0.35, 0.39],
  brace: [0.52, 0.56, 0.62],
};

/** 沿 x 轴的盒子：x 中心与长度、y 中心与高、z 中心与宽 */
function boxX(xc, len, yc, h, zc, w, color) {
  return transformed(boxTris(len, h, w, color), [xc, yc, zc]);
}
/** 轴沿 x 的圆柱 */
function cylX(radius, len, color, seg, center) {
  return rotateYThenTranslate(cylinderTris(radius, len, color, seg), Math.PI / 2, center);
}
/** 由包围盒 [x1,y1,z1,x2,y2,z2] 生成盒子 */
function boxFromBounds(b, color) {
  const sx = Math.max(1e-4, b[3] - b[0]);
  const sy = Math.max(1e-4, b[4] - b[1]);
  const sz = Math.max(1e-4, b[5] - b[2]);
  return transformed(boxTris(sx, sy, sz, color), [(b[0] + b[3]) / 2, (b[1] + b[4]) / 2, (b[2] + b[5]) / 2]);
}

/** 方截面斜撑：两点之间的方形杆（45° 斜撑用它，包围盒画会变成整块板） */
function rodBoxTris(a, b, sec, color) {
  const d = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
  const len = Math.hypot(d[0], d[1], d[2]) || 1e-9;
  const y = [d[0] / len, d[1] / len, d[2] / len];
  const ref = Math.abs(y[1]) < 0.9 ? [0, 1, 0] : [0, 0, 1];
  const x = [
    y[1] * ref[2] - y[2] * ref[1],
    y[2] * ref[0] - y[0] * ref[2],
    y[0] * ref[1] - y[1] * ref[0],
  ];
  const xl = Math.hypot(x[0], x[1], x[2]) || 1;
  x[0] /= xl; x[1] /= xl; x[2] /= xl;
  const z = [
    y[1] * x[2] - y[2] * x[1],
    y[2] * x[0] - y[0] * x[2],
    y[0] * x[1] - y[1] * x[0],
  ];
  const base = boxTris(sec, len, sec, color);
  const mid = [(a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2];
  return base.map((t) => ({
    pos: [
      t.pos[0] * x[0] + t.pos[1] * y[0] + t.pos[2] * z[0] + mid[0],
      t.pos[0] * x[1] + t.pos[1] * y[1] + t.pos[2] * z[1] + mid[1],
      t.pos[0] * x[2] + t.pos[1] * y[2] + t.pos[2] * z[2] + mid[2],
    ],
    nrm: [
      t.nrm[0] * x[0] + t.nrm[1] * y[0] + t.nrm[2] * z[0],
      t.nrm[0] * x[1] + t.nrm[1] * y[1] + t.nrm[2] * z[1],
      t.nrm[0] * x[2] + t.nrm[1] * y[2] + t.nrm[2] * z[2],
    ],
    col: t.col,
  }));
}

/**
 * 8 个卷筒 / 锚点位置，与 sim/core 的 buildModel 完全一致。
 *
 * 两种拓扑：
 *   topology === 'corner'：严格 8 角 —— 锚点在框架 4 个角上（上下各一台），
 *     系绳点在动平台 4 个角上（上下表面各一个）。i = 2j + k
 *     （j 为角：+x+z, +x-z, -x-z, -x+z；k=0 上绳 / 1 下绳）。
 *   其他（edge）：锚点在「四边中点」的法向 half_spacing 处，沿边微错开 ±edge_offset。
 */
export function winderLayout(p) {
  const w = p.winders;
  const f = p.frames;
  const out = [];
  // 四条边：(法向, 切向) —— 与 core.js 的 edges 一致
  const edges = [
    { n: [1, 0], t: [0, 1] },    // +x 边
    { n: [0, 1], t: [-1, 0] },   // +z 边
    { n: [-1, 0], t: [0, -1] },  // -x 边
    { n: [0, -1], t: [1, 0] },   // -z 边
  ];

  if (w.topology === 'corner') {
    // 严格 8 角：4 个水平角 × 上下两层
    const S = w.corner_spacing ?? 0.485;
    const corners = [[1, 1], [1, -1], [-1, -1], [-1, 1]];
    const mUpY = w.motor_upper_y ?? w.upper_y;
    const mLoY = w.motor_lower_y ?? w.lower_y;
    for (let j = 0; j < 4; j++) {
      const [sx, sz] = corners[j];
      for (let k = 0; k < 2; k++) {
        const y = k === 0 ? w.upper_y : w.lower_y;       // 锚点高度（力学）
        const mY = k === 0 ? mUpY : mLoY;                // 电机安装高度（外观）
        const ax = sx * S;
        const az = sz * S;
        // 电机装在角的外侧（沿对角线向外推），卷筒朝内
        const dx = sx / Math.SQRT2, dz = sz / Math.SQRT2;
        const mx = ax + dx * 0.10;
        const mz = az + dz * 0.10;
        out.push({
          index: 2 * j + k,
          pair: j,
          level: k,
          anchor: [ax, y, az],
          motorPos: [mx, mY, mz],
          normal: [dx, dz],
          tangent: [dz, -dx],
          topology: 'corner',
          leadFromDrum: Math.abs(mY - y) > 1e-6,
          outside: Math.hypot(mx, mz) > f.width / 2 - 0.06,
        });
      }
    }
    return out;
  }

  const S = w.half_spacing_x;
  const off = w.edge_offset ?? 0.04;
  // 电机「显示位置」：装了 corner_motor_display 时摆到框架 8 角（仅外观，
  // 不参与力学 —— 绳的锚点仍在四边中点，卷筒经一段短绳引到锚点）。
  const showAtCorner = !!w.corner_motor_display;
  const CS = w.corner_spacing ?? 0.485;
  // 下组电机的「安装高度」：可以低于锚点（电机放框架底部，经导轮把绳引上去）。
  // motor_lower_y 未设置时与锚点同高。
  const motorLoY = w.motor_lower_y ?? w.lower_y;
  const motorUpY = w.motor_upper_y ?? w.upper_y;
  for (let j = 0; j < 4; j++) {
    const e = edges[j];
    const nrm = [e.n[0], e.n[1]];
    for (let k = 0; k < 2; k++) {
      const s = k === 0 ? 1 : -1;
      const y = k === 0 ? w.upper_y : w.lower_y;          // 锚点高度（力学）
      const mY = k === 0 ? motorUpY : motorLoY;           // 电机高度（外观）
      const anchor = [e.n[0] * S + e.t[0] * s * off, y, e.n[1] * S + e.t[1] * s * off];
      // 电机位置：默认沿法向外移；开启角点显示时挪到该边的角上
      let motorPos;
      if (showAtCorner) {
        const co = w.corner_offset ?? 0.10;
        motorPos = [
          e.n[0] * CS + e.t[0] * s * co,
          mY,
          e.n[1] * CS + e.t[1] * s * co,
        ];
      } else {
        motorPos = [anchor[0] + nrm[0] * 0.085, mY, anchor[2] + nrm[1] * 0.085];
      }
      out.push({
        index: 2 * j + k,
        pair: j,
        level: k,
        anchor,
        motorPos,
        normal: nrm,
        tangent: [e.t[0], e.t[1]],
        showAtCorner,
        // 电机高度与锚点不同时，需要一段「引绳」（卷筒 -> 锚点）
        leadFromDrum: Math.abs(mY - y) > 1e-6,
        outside: Math.abs(e.n[0] * S) + 0.085 > f.width / 2 - 0.02,
      });
    }
  }
  return out;
}

export class Scene {
  constructor(renderer, params) {
    this.r = renderer;
    this.params = params;
    this.show = { frame: true, motors: true, cables: true, grid: true, target: true, forces: false };
    this.colorMode = 'tension';
    this.buildStatic();
    this.buildPlatform();
  }

  setParams(p) {
    this.params = p;
    this.buildStatic();
    this.buildPlatform();
  }

  // ------------------------------------------------------------ 静态结构
  buildStatic() {
    const r = this.r;
    const p = this.params;
    const f = p.frames;
    const tris = [];

    // ---- 框架：SolidWorks 实测 44 根 30×30 铝型材（28 直杆 + 16 根 45° 斜撑）----
    if (Array.isArray(f.parts) && f.parts.length) {
      for (const it of f.parts) {
        if (it.t === 'rod') {
          tris.push(...rodBoxTris(it.a, it.b, it.s ?? 0.03, C.brace));
        } else {
          const b = it.b;
          const dy = b[4] - b[1];
          const dx = b[3] - b[0];
          const dz = b[5] - b[2];
          // 竖直立柱 → 常规金属色；长横梁 → 稍亮；短接头 → 中间色
          let col = C.metal;
          if (dy > 0.6) col = C.metal;
          else if (dx > 0.6 || dz > 0.6) col = C.brace;
          else col = C.bracket;
          tris.push(...boxFromBounds(b, col));
        }
      }
    } else {
      // 兜底：没有型材表时按外廓画简化框
      const W = f.width, D = f.depth, H = f.top_y, prof = f.profile ?? 0.03;
      const cx = W / 2 - prof / 2, cz = D / 2 - prof / 2;
      for (const sx of [-1, 1]) for (const sz of [-1, 1]) {
        tris.push(...boxX(sx * cx, prof, (H + 0.025) / 2, H - 0.025, sz * cz, prof, C.metal));
      }
      tris.push(...boxX(0, W, H - prof / 2, prof, cz, prof, C.metal));
      tris.push(...boxX(0, W, H - prof / 2, prof, -cz, prof, C.metal));
      tris.push(...boxX(cx, prof, H - prof / 2, prof, 0, D, C.metal));
      tris.push(...boxX(-cx, prof, H - prof / 2, prof, 0, D, C.metal));
    }

    // ---- 两块夹层板（上夹层 / 下夹层，实测 988×988×10 mm）----
    //
    // 上夹层板必须开四角缺口：力学锚点 y = winders.upper_y = 1.10 高于板面 0.985，
    // 上组 4 根绳从角上的锚点斜向下走到平台四角，一定穿过 y = 0.985 平面。
    // 缺口尺寸 deck_hi_notch 由「全部可达姿态下绳-板交点」扫出来（见 docs/参数表.md §1.1）：
    // 交点恒落在 (|x|>0.19, |z|>0.19) 的四角区域内，故把四个角各切掉一块方形，
    // 板面变成十字形，绳从缺口内通过，不再穿模。
    const deckHalf = (f.deck_size ?? 0.9884) / 2;
    const deckT = f.deck_thickness ?? 0.01;
    const deckLoY = f.deck_lo_y ?? 0.015;
    tris.push(...boxX(0, deckHalf * 2, deckLoY + deckT / 2, deckT, 0, deckHalf * 2, C.deck));
    const deckHiY = f.deck_hi_y ?? 0.985;
    const notch = Math.min(f.deck_hi_notch ?? 0, deckHalf * 0.98);
    const cy = deckHiY + deckT / 2;
    if (notch <= 1e-4) {
      tris.push(...boxX(0, deckHalf * 2, cy, deckT, 0, deckHalf * 2, C.deck));
    } else {
      // 十字形板：中带（x 全长、z ∈ [−notch, notch]）+ 两条 z 向臂
      tris.push(...boxX(0, deckHalf * 2, cy, deckT, 0, notch * 2, C.deck));
      for (const s of [-1, 1]) {
        tris.push(...boxX(0, notch * 2, cy, deckT, s * (deckHalf + notch) / 2, deckHalf - notch, C.deck));
      }
    }

    this.winders = winderLayout(p);
    const isCorner = p.winders.topology === 'corner';
    // 电机座：标准拓扑用小托板；角点拓扑用一块方形角座
    for (const wd of this.winders) {
      const [nx, nz] = wd.normal;
      if (isCorner) {
        // 角座：贴在立柱内侧，承托电机
        const [tx, tz] = wd.tangent ?? [0, 0];
        const cx = wd.anchor[0] - nx * 0.11 - tx * (wd.level === 0 ? 1 : -1) * 0.02;
        const cz = wd.anchor[2] - nz * 0.11 - tz * (wd.level === 0 ? 1 : -1) * 0.02;
        tris.push(...boxX(cx, 0.20, wd.anchor[1] - 0.055, 0.008, cz, 0.20, C.bracket));
      } else {
        // 托板跟着「电机安装高度」走（可能比锚点低很多）
        tris.push(...boxX(
          wd.motorPos[0] - nx * 0.10, 0.30,
          wd.motorPos[1] - 0.052, 0.008,
          wd.motorPos[2] - nz * 0.10, 0.30, C.plate,
        ));
        // 电机装在框架底部时，额外画一根竖直支撑（从底板顶到电机托板）
        const f0 = p.frames;
        const floorY = (f0.deck_lo_y ?? 0.015) + (f0.deck_thickness ?? 0.01);
        if (wd.motorPos[1] - floorY > 0.12) {
          const h = wd.motorPos[1] - 0.056 - floorY;
          const cx = wd.motorPos[0] - nx * 0.10;
          const cz = wd.motorPos[2] - nz * 0.10;
          for (const [ox, oz] of [[-0.11, -0.11], [0.11, -0.11], [-0.11, 0.11], [0.11, 0.11]]) {
            tris.push(...boxX(cx + ox, 0.024, floorY + h / 2, h, cz + oz, 0.024, C.bracket));
          }
        }
      }
    }
    this.staticMesh = r.createMesh(tris);

    // ---- 8 套电机：机身 + 编码器 + 减速器 + 卷筒 + 出绳滑轮 ----
    const m = [];
    const drumR = p.winders.drum_radius;
    for (const wd of this.winders) {
      const [mx, my, mz] = wd.motorPos;
      const [nx, nz] = wd.normal;
      // 角点拓扑：电机沿「法向」朝外，卷筒朝内
      const outX = nx, outZ = nz;
      const inX = -nx, inZ = -nz;
      if (isCorner) {
        // 机身短一些（角上空间有限），沿法向摆放
        m.push(...boxX(mx + outX * 0.06, 0.12, my, 0.086, mz + outZ * 0.06, 0.086, C.motor));
        m.push(...boxX(mx + outX * 0.125, 0.025, my, 0.072, mz + outZ * 0.125, 0.072, C.motorCap));
        // 减速器 + 卷筒（朝内，指向平台）
        m.push(...cylX(0.042, 0.05, C.gear, 12, [mx + inX * 0.04, my, mz + inZ * 0.04]));
        m.push(...cylX(drumR, 0.05, C.drum, 12, [mx + inX * 0.095, my, mz + inZ * 0.095]));
        m.push(...cylX(drumR + 0.006, 0.005, C.metalDark, 12, [mx + inX * 0.072, my, mz + inZ * 0.072]));
        m.push(...cylX(drumR + 0.006, 0.005, C.metalDark, 12, [mx + inX * 0.118, my, mz + inZ * 0.118]));
      } else if (wd.showAtCorner) {
        // 电机显示在框架角上（外观），但卷筒方向指向「四边中点」的锚点，
        // 这样从卷筒到锚点的引绳段短且自然。
        // 若电机装在框架底部（motor_lower_y 远低于锚点），卷筒还要带一个仰角，
        // 让引绳朝上引到锚点。
        const ax = wd.anchor[0] - mx, az = wd.anchor[2] - mz;
        const ay = wd.anchor[1] - my;
        const al = Math.hypot(ax, az) || 1;
        const dx = ax / al, dz = az / al;
        // 机身沿「背离锚点」的水平方向摆；卷筒朝锚点
        m.push(...boxX(mx - dx * 0.075, 0.15, my, 0.086, mz - dz * 0.075, 0.086, C.motor));
        m.push(...boxX(mx - dx * 0.155, 0.03, my, 0.072, mz - dz * 0.155, 0.072, C.motorCap));
        // 减速器 + 卷筒 + 两侧挡板（朝锚点）
        m.push(...cylX(0.045, 0.06, C.gear, 12, [mx + dx * 0.045, my, mz + dz * 0.045]));
        m.push(...cylX(drumR + 0.006, 0.005, C.metalDark, 12, [mx + dx * 0.075, my, mz + dz * 0.075]));
        m.push(...cylX(drumR + 0.006, 0.005, C.metalDark, 12, [mx + dx * 0.135, my, mz + dz * 0.135]));
        m.push(...cylX(drumR, 0.05, C.drum, 12, [mx + dx * 0.09, my, mz + dz * 0.09]));
        // 电机装在框架底部时，卷筒到锚点之间加一个「出绳导轮」，示意绳向上引
        if (Math.abs(ay) > 0.05) {
          m.push(...cylX(0.014, 0.010, C.pulley, 10, [mx + dx * 0.145, my + 0.02, mz + dz * 0.145]));
        }
      } else {
        m.push(...boxX(mx + outX * 0.075, 0.15, my, 0.086, mz + outZ * 0.075, 0.086, C.motor));
        m.push(...boxX(mx + outX * 0.155, 0.03, my, 0.072, mz + outZ * 0.155, 0.072, C.motorCap));
        m.push(...cylX(0.045, 0.06, C.gear, 12, [mx + inX * 0.045, my, mz + inZ * 0.045]));
        m.push(...cylX(drumR, 0.06, C.drum, 12, [mx + inX * 0.105, my, mz + inZ * 0.105]));
        m.push(...cylX(drumR + 0.006, 0.005, C.metalDark, 12, [mx + inX * 0.075, my, mz + inZ * 0.075]));
        m.push(...cylX(drumR + 0.006, 0.005, C.metalDark, 12, [mx + inX * 0.135, my, mz + inZ * 0.135]));
      }
      // 出绳滑轮（锚点）
      m.push(...cylX(isCorner ? 0.034 : 0.014, isCorner ? 0.014 : 0.010, C.pulley, 10, wd.anchor));
    }
    this.motorMesh = r.createMesh(m);
    // 卷筒出绳点（世界坐标）：卷筒靠内的一端
    this.drumPoints = this.winders.map((wd) => {
      const [nx, nz] = wd.normal;
      if (wd.showAtCorner && !isCorner) {
        // 显示在角上：出绳点朝锚点方向
        const ax = wd.anchor[0] - wd.motorPos[0], az = wd.anchor[2] - wd.motorPos[2];
        const al = Math.hypot(ax, az) || 1;
        return [wd.motorPos[0] + (ax / al) * 0.135, wd.motorPos[1], wd.motorPos[2] + (az / al) * 0.135];
      }
      const d = isCorner ? 0.118 : 0.135;
      return [wd.motorPos[0] - nx * d, wd.motorPos[1], wd.motorPos[2] - nz * d];
    });
  }

  // ------------------------------------------------------------ 动平台
  /**
   * 8 个系绳点的平台局部坐标，与 sim/core 的 buildModel 完全一致：
   *   i = 2j + k（j 为角：+x+z, +x-z, -x-z, -x+z；k=0 上绳 / 1 下绳）
   *
   * 严格 8 角拓扑：系绳点在平台 4 个角上，上绳接上表面、下绳接下表面。
   * 标准拓扑：每边两个角点，上绳接一角上表面、下绳接另一角下表面。
   *
   * 顺序若与 core 不一致，三维画面里的绳索就会连错点（看起来交叉打结）。
   */
  modelPoints() {
    const p = this.params;
    const w = p.winders;
    const hx = p.platform.half_size_x, hz = p.platform.half_size_z, t = p.platform.thickness;
    const pts = [];
    if (w.topology === 'corner') {
      // 4 个角 × 上下表面，顺序与 core.js 的 corners 一致
      const corners = [[1, 1], [1, -1], [-1, -1], [-1, 1]];
      for (const [sx, sz] of corners) {
        pts.push([sx * hx, t / 2, sz * hz]);
        pts.push([sx * hx, -t / 2, sz * hz]);
      }
      return pts;
    }
    const edges = [
      { n: [1, 0], t: [0, 1] },
      { n: [0, 1], t: [-1, 0] },
      { n: [-1, 0], t: [0, -1] },
      { n: [0, -1], t: [1, 0] },
    ];
    const edgeCorners = [
      [[1, 1], [1, -1]],
      [[1, 1], [-1, 1]],
      [[-1, 1], [-1, -1]],
      [[1, -1], [-1, -1]],
    ];
    for (let j = 0; j < 4; j++) {
      const [[ax, az], [bx, bz]] = edgeCorners[j];
      pts.push([ax * hx, t / 2, az * hz]);
      pts.push([bx * hx, -t / 2, bz * hz]);
    }
    return pts;
  }

  buildPlatform() {
    const r = this.r;
    const p = this.params;
    const hx = p.platform.half_size_x, hz = p.platform.half_size_z, t = p.platform.thickness;
    const tris = [];
    const isCorner = p.winders.topology === 'corner';
    // 上平台板
    tris.push(...boxX(0, 2 * hx, t * 0.32, t * 0.36, 0, 2 * hz, C.platform));
    // 下层板
    tris.push(...boxX(0, 2 * hx * 0.86, -t * 0.3, t * 0.3, 0, 2 * hz * 0.86, C.platform2));
    // 连接柱
    tris.push(...boxX(0, 2 * hx * 0.4, 0, t * 0.7, 0, 2 * hz * 0.4, C.tie));
    if (isCorner) {
      // 角点拓扑：系绳点在四边中点，用小的系绳座标出
      for (const b of this.modelPoints()) {
        const col = b[1] > 0 ? [0.92, 0.22, 0.22] : [0.25, 0.45, 0.95];
        tris.push(...transformed(boxTris(0.014, 0.014, 0.014, col), b));
      }
    } else {
      for (const b of this.modelPoints()) {
        const col = b[1] > 0 ? [0.92, 0.22, 0.22] : [0.25, 0.45, 0.95];
        tris.push(...transformed(boxTris(0.01, 0.01, 0.01, col), b));
      }
    }
    this.platformMesh = r.createMesh(tris);
  }

  tensionColor(T, lo, hi) {
    const x = Math.min(1, Math.max(0, (T - lo) / Math.max(1e-6, hi - lo)));
    if (x < 0.5) {
      const s = x / 0.5;
      return [0.25 + 0.1 * s, 0.55 + 0.45 * s, 1.0 - 0.65 * s];
    }
    const s = (x - 0.5) / 0.5;
    return [0.35 + 0.65 * s, 1.0 - 0.65 * s, 0.35 - 0.3 * s];
  }

  // ------------------------------------------------------------ 每帧
  update(snap) {
    if (!snap) return;
    const r = this.r;
    const p = this.params;
    const localPts = this.modelPoints();
    const R = poseMatrix(snap.q);
    const worldPts = localPts.map((b) => applyMat(R, b));
    this.platformModel = R;
    this.platformWorldPoints = worldPts;
    const T = snap.T || [];
    const hi = Math.max(p.cables.t_max * 0.2, ...T.map((v) => v));
    const pos = [];
    const col = [];
    for (const wd of this.winders) {
      const i = wd.index;
      const drum = this.drumPoints[i];
      const a = wd.anchor;
      const b = worldPts[i];
      const c = this.colorMode === 'tension' ? this.tensionColor(T[i] || 0, 0, hi) : [0.05, 0.05, 0.06];
      pos.push(...drum, ...a, ...a, ...b);
      col.push(...c, ...c, ...c, ...c);
    }
    r.updateLines('cables', pos, col);
    const RT = poseMatrix(snap.qTarget || snap.q);
    const tpT = localPts.map((b) => applyMat(RT, b));
    const edges = [[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6], [6, 7], [7, 4], [0, 4], [1, 5], [2, 6], [3, 7]];
    const lpos = [];
    const lcol = [];
    for (const [a, b] of edges) {
      lpos.push(...tpT[a], ...tpT[b]);
      lcol.push(0.3, 0.95, 1.0, 0.3, 0.95, 1.0);
    }
    r.updateLines('target', lpos, lcol);

    if (this.show.forces) {
      const fpos = [];
      const fcol = [];
      for (const wd of this.winders) {
        const i = wd.index;
        const b = worldPts[i];
        const a = wd.anchor;
        const d = [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
        const l = Math.hypot(d[0], d[1], d[2]) || 1;
        const len = Math.min(0.12, ((T[i] || 0) / 20) * 0.05);
        const tip = [b[0] + (d[0] / l) * len, b[1] + (d[1] / l) * len, b[2] + (d[2] / l) * len];
        fpos.push(...b, ...tip);
        const c = this.tensionColor(T[i] || 0, 0, hi);
        fcol.push(...c, ...c);
      }
      r.updateLines('forces', fpos, fcol);
    }
  }

  render(camera) {
    const r = this.r;
    r.begin([0.075, 0.085, 0.105]);
    if (this.show.grid) {
      this.updateGround();
      r.drawLines('grid', camera, 1);
    }
    // 框架 + 电机是世界坐标里静止的几何：用离屏缓存（相机不变时整图贴回），
    // 避免每帧对 ~4k 三角形重新投影/排序/填充。这是帧率优化的核心。
    if (this.show.frame || this.show.motors) {
      const meshes = [];
      if (this.show.frame) meshes.push(this.staticMesh);
      if (this.show.motors) meshes.push(this.motorMesh);
      r.drawStatic(meshes, camera);
    }
    if (this.platformModel) r.drawMesh(this.platformMesh, this.platformModel, camera);
    r.flush();
    if (this.show.cables) r.drawLines('cables', camera, 2);
    if (this.show.target) r.drawLines('target', camera, 1.2);
    if (this.show.forces) r.drawLines('forces', camera, 3);
  }

  updateGround() {
    const r = this.r;
    const f = this.params.frames;
    const pos = [];
    const col = [];
    const ext = Math.max(f.width, f.depth) * 0.8;
    const step = 0.1;
    for (let x = -ext; x <= ext + 1e-6; x += step) {
      pos.push(x, 0.0008, -ext, x, 0.0008, ext);
      const c = Math.abs(x) < 1e-6 ? [0.42, 0.45, 0.5] : [0.17, 0.19, 0.23];
      col.push(...c, ...c);
    }
    for (let z = -ext; z <= ext + 1e-6; z += step) {
      pos.push(-ext, 0.0008, z, ext, 0.0008, z);
      const c = Math.abs(z) < 1e-6 ? [0.42, 0.45, 0.5] : [0.17, 0.19, 0.23];
      col.push(...c, ...c);
    }
    r.updateLines('grid', pos, col);
  }
}

export { OrbitCamera, poseMatrix, applyMat };
