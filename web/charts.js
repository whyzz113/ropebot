/**
 * 轻量实时曲线：张力/绳长随时间变化（Canvas 2D，无第三方依赖）。
 */
const PALETTE = [
  '#ef5350', '#ffa726', '#ffd54f', '#66bb6a',
  '#26c6da', '#42a5f5', '#ab47bc', '#ec407a',
];

export class TimeChart {
  constructor(canvas, opts = {}) {
    this.canvas = canvas;
    this.ctx = canvas.getContext('2d');
    this.series = Array.from({ length: opts.count ?? 8 }, () => []);
    this.windowSec = opts.windowSec ?? 10;
    this.yUnit = opts.yUnit ?? '';
    this.title = opts.title ?? '';
    this.t = [];
    this.hidden = new Set();
    this.dpr = Math.min(window.devicePixelRatio || 1, 2);
  }

  push(t, values) {
    this.t.push(t);
    for (let i = 0; i < this.series.length; i++) this.series[i].push(values[i]);
    const cut = t - this.windowSec;
    while (this.t.length && this.t[0] < cut) {
      this.t.shift();
      for (const s of this.series) s.shift();
    }
  }

  toggle(i) {
    if (this.hidden.has(i)) this.hidden.delete(i);
    else this.hidden.add(i);
  }

  draw() {
    const cv = this.canvas;
    const w = cv.clientWidth, h = cv.clientHeight;
    if (cv.width !== w * this.dpr || cv.height !== h * this.dpr) {
      cv.width = w * this.dpr;
      cv.height = h * this.dpr;
    }
    const ctx = this.ctx;
    ctx.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    ctx.clearRect(0, 0, w, h);
    const padL = 40, padR = 62, padT = 18, padB = 16;
    const iw = w - padL - padR, ih = h - padT - padB;
    // 背景
    ctx.fillStyle = 'rgba(20,24,30,0.75)';
    ctx.fillRect(padL, padT, iw, ih);
    // y 范围
    let lo = Infinity, hi = -Infinity;
    for (let i = 0; i < this.series.length; i++) {
      if (this.hidden.has(i)) continue;
      for (const v of this.series[i]) {
        if (v < lo) lo = v;
        if (v > hi) hi = v;
      }
    }
    if (!isFinite(lo)) { lo = 0; hi = 1; }
    const span = Math.max(hi - lo, 1e-6);
    lo -= span * 0.1;
    hi += span * 0.1;
    const yOf = (v) => padT + ih - ((v - lo) / (hi - lo)) * ih;
    const t0 = this.t.length ? this.t[0] : 0;
    const t1 = this.t.length ? this.t[this.t.length - 1] : 1;
    const xOf = (t) => padL + ((t - t0) / Math.max(t1 - t0, 1e-6)) * iw;
    // 网格
    ctx.strokeStyle = 'rgba(255,255,255,0.08)';
    ctx.lineWidth = 1;
    for (let k = 0; k <= 4; k++) {
      const y = padT + (ih * k) / 4;
      ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(padL + iw, y); ctx.stroke();
      const v = lo + ((hi - lo) * (4 - k)) / 4;
      ctx.fillStyle = 'rgba(200,210,220,0.75)';
      ctx.font = '10px ui-monospace, Consolas, monospace';
      ctx.fillText(v.toFixed(v > 100 ? 0 : 1), 4, y + 3);
    }
    // 曲线
    for (let i = 0; i < this.series.length; i++) {
      if (this.hidden.has(i)) continue;
      ctx.strokeStyle = PALETTE[i % PALETTE.length];
      ctx.lineWidth = 1.4;
      ctx.beginPath();
      const s = this.series[i];
      for (let k = 0; k < s.length; k++) {
        const x = xOf(this.t[k]), y = yOf(s[k]);
        if (k === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }
    // 标题 & 单位
    ctx.fillStyle = 'rgba(235,240,245,0.95)';
    ctx.font = '11px system-ui, "Microsoft YaHei", sans-serif';
    ctx.fillText(this.title, padL, 12);
    if (this.t.length) {
      const last = this.t[this.t.length - 1];
      ctx.fillStyle = 'rgba(160,170,180,0.9)';
      ctx.fillText(`${last.toFixed(1)} s`, padL + iw - 34, 12);
    }
  }

  /** 图例 HTML（点击可隐藏曲线） */
  legendHtml(labels) {
    return labels
      .map((lab, i) => `<span class="lg${this.hidden.has(i) ? ' off' : ''}" data-i="${i}" style="--c:${PALETTE[i % PALETTE.length]}">${lab}</span>`)
      .join('');
  }
}

export { PALETTE };
