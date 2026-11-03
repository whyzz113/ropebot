/**
 * 数值内核：线性方程组求解与最小范数/最小二乘。
 *
 * solveDense(M, b)   : 方阵 M x = b（列主元高斯消元，带相对奇异判据）
 * minNormSolve(M, b) : M 为 m x n（m 行方程、n 个未知数）
 *                      m < n（欠定，如 A 6x8）-> min||x|| s.t. M x = b
 *                      m >= n（超定，如 A^T 8x6）-> min||M x - b||
 * 实现：在 Gram 矩阵 G = M M^T 上做高斯-约当消元得到 G^-1，再 x = M^T G^-1 b。
 * G 半正定，列主元等价按奇异值从大到小主元；秩亏时小主元置零，
 * 得到截断伪逆解（冗余驱动、奇异姿态下也不会出现 NaN/爆炸）。
 */
import { matNew } from './core.js';

export function solveDense(M, b) {
  const n = M.rows;
  if (M.cols !== n) throw new Error(`solveDense 需要方阵，收到 ${M.rows}x${M.cols}`);
  const stride = M.cols;
  const a = Float64Array.from(M.d);
  const x = Float64Array.from(b);
  let amax = 0;
  for (let i = 0; i < a.length; i++) amax = Math.max(amax, Math.abs(a[i]));
  const tiny = Math.max(amax * 1e-14, 1e-300);
  for (let c = 0; c < n; c++) {
    let piv = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(a[r * stride + c]) > Math.abs(a[piv * stride + c])) piv = r;
    if (Math.abs(a[piv * stride + c]) < tiny) continue;
    if (piv !== c) {
      for (let k = 0; k < n; k++) {
        const t = a[c * stride + k];
        a[c * stride + k] = a[piv * stride + k];
        a[piv * stride + k] = t;
      }
      const t = x[c]; x[c] = x[piv]; x[piv] = t;
    }
    const d = a[c * stride + c];
    for (let r = c + 1; r < n; r++) {
      const f = a[r * stride + c] / d;
      if (f === 0) continue;
      for (let k = c; k < n; k++) a[r * stride + k] -= f * a[c * stride + k];
      x[r] -= f * x[c];
    }
  }
  for (let r = n - 1; r >= 0; r--) {
    let s = x[r];
    for (let k = r + 1; k < n; k++) s -= a[r * stride + k] * x[k];
    const d = a[r * stride + r];
    x[r] = Math.abs(d) < tiny ? 0 : s / d;
  }
  return x;
}

export function minNormSolve(M, b) {
  const m = M.rows;
  const n = M.cols;
  if (b.length !== m) throw new Error(`minNormSolve 维度不匹配：M ${m}x${n}, b ${b.length}`);
  const G = matNew(m, m);
  for (let i = 0; i < m; i++)
    for (let j = 0; j < m; j++) {
      let s = 0;
      for (let k = 0; k < n; k++) s += M.d[i * n + k] * M.d[j * n + k];
      G.d[i * m + j] = s;
    }
  const inv = matNew(m, m);
  for (let i = 0; i < m; i++) inv.d[i * m + i] = 1;
  let gmax = 0;
  for (let i = 0; i < m * m; i++) gmax = Math.max(gmax, Math.abs(G.d[i]));
  if (gmax === 0) return new Float64Array(n);
  const tiny = gmax * 1e-12;
  for (let c = 0; c < m; c++) {
    let piv = c;
    for (let r = c + 1; r < m; r++) if (Math.abs(G.d[r * m + c]) > Math.abs(G.d[piv * m + c])) piv = r;
    if (Math.abs(G.d[piv * m + c]) < tiny) continue;
    if (piv !== c) {
      for (let k = 0; k < m; k++) {
        const t = G.d[c * m + k]; G.d[c * m + k] = G.d[piv * m + k]; G.d[piv * m + k] = t;
        const u = inv.d[c * m + k]; inv.d[c * m + k] = inv.d[piv * m + k]; inv.d[piv * m + k] = u;
      }
    }
    const d = G.d[c * m + c];
    for (let k = 0; k < m; k++) {
      G.d[c * m + k] /= d;
      inv.d[c * m + k] /= d;
    }
    for (let r = 0; r < m; r++) {
      if (r === c) continue;
      const f = G.d[r * m + c];
      if (f === 0) continue;
      for (let k = 0; k < m; k++) {
        G.d[r * m + k] -= f * G.d[c * m + k];
        inv.d[r * m + k] -= f * inv.d[c * m + k];
      }
    }
  }
  const y = new Float64Array(m);
  for (let i = 0; i < m; i++) {
    let s = 0;
    for (let j = 0; j < m; j++) s += inv.d[i * m + j] * b[j];
    y[i] = Number.isFinite(s) ? s : 0;
  }
  const x = new Float64Array(n);
  for (let k = 0; k < n; k++) {
    let s = 0;
    for (let i = 0; i < m; i++) s += M.d[i * n + k] * y[i];
    x[k] = Number.isFinite(s) ? s : 0;
  }
  return x;
}
