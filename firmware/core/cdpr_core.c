/**
 * 绳驱并联机器人控制核心实现（C）
 * 与 sim/core/*.js 一一对应，可在 STM32F407 与 PC 板级仿真中共用。
 */
#include "cdpr_core.h"
/* 拓扑宏 CDPR_TOPOLOGY_CORNER 定义在 cdpr_params.h 里；
 * cdpr_core.c 必须自己 include，否则本文件内的 #if 会取默认值 0，
 * 导致 C 侧几何与 JS 侧（按 JSON 的 topology 字段）不一致。 */
#include "cdpr_params.h"
#include <math.h>
#include <string.h>

#define PI_F 3.14159265358979f

/* 8 个锚点：i = 2j + k
 *   j = 0..3 为四条边：+x, +z, -x, -z
 *   k = 0 上绳（anchor_upper_y）/ 1 下绳（anchor_lower_y）
 *
 * 角点拓扑（CDPR_TOPO_CORNER，与 sim/core/core.js 的 topology:'corner' 一致）：
 *   锚点在框架四个角上（法向 anchor_x），上绳取该边 +t 侧的角、
 *   下绳取 -t 侧的角，沿边错开 corner_offset。
 *
 * 标准拓扑（默认）：锚点在「边中点」的法向 anchor_x 处，沿边按 edge_offset 微错开。 */
static void anchor_of(const cdpr_ctl_t *c, int i, float out[3]) {
    const int j = i >> 1;
    const int k = i & 1;
    const float s = (k == 0) ? 1.0f : -1.0f;
    float nx, nz, tx, tz;
    /* 标准拓扑的切向必须与 sim/core/core.js 的 `tu` 一致：
     *   nx != 0 的边（±x）→ tu = (0, 1)
     *   nx == 0 的边（±z）→ tu = (1, 0)
     * 即 j = 0/2 用 (0,1)，j = 1/3 用 (1,0)。
     * 角点拓扑另有一套切向（见下方 CDPR_TOPOLOGY_CORNER 分支），
     * 与 core.js 的 edges[].t 一致：+x→(0,1) +z→(-1,0) -x→(0,-1) -z→(1,0)。 */
    switch (j) {
        case 0: nx = 1.0f;  nz = 0.0f;  tx = 0.0f;  tz = 1.0f;  break; /* +x 边 */
        case 1: nx = 0.0f;  nz = 1.0f;  tx = 1.0f;  tz = 0.0f;  break; /* +z 边 */
        case 2: nx = -1.0f; nz = 0.0f;  tx = 0.0f;  tz = 1.0f;  break; /* -x 边 */
        default: nx = 0.0f; nz = -1.0f; tx = 1.0f;  tz = 0.0f;  break; /* -z 边 */
    }
#if CDPR_TOPOLOGY_CORNER
    /* 严格 8 角：锚点在框架 4 个水平角上，上绳 upper_y / 下绳 lower_y。
     * 与 core.js 的 corners 一致：
     *   j=0 → (+1,+1)  j=1 → (+1,-1)  j=2 → (-1,-1)  j=3 → (-1,+1) */
    float cx, cz;
    switch (j) {
        case 0: cx = 1.0f;  cz = 1.0f;  break;
        case 1: cx = 1.0f;  cz = -1.0f; break;
        case 2: cx = -1.0f; cz = -1.0f; break;
        default: cx = -1.0f; cz = 1.0f; break;
    }
    const float S = c->p.corner_spacing;
    out[0] = cx * S;
    out[1] = (k == 0) ? c->p.anchor_upper_y : c->p.anchor_lower_y;
    out[2] = cz * S;
#else
    const float S = c->p.anchor_x;
    const float off = c->p.edge_offset;
    out[0] = nx * S + tx * s * off;
    out[1] = (k == 0) ? c->p.anchor_upper_y : c->p.anchor_lower_y;
    out[2] = nz * S + tz * s * off;
#endif
}

/* 平台系绳点（平台坐标系），与锚点一一对应。
 *
 * 标准拓扑：每条边的两个角点 —— 上绳接该边第一个角点的上表面，
 *   下绳接该边第二个角点的下表面（同边同侧，俯视不交叉）。
 *
 * 角点拓扑：每条边的「中点」—— 上绳接上表面、下绳接下表面，
 *   沿边错开 ±tether_offset。这样上下两组绳的水平投影方向不同，
 *   结构矩阵的俯仰力矩行不会退化。 */
static void point_of(const cdpr_ctl_t *c, int i, float out[3]) {
    const int j = i >> 1;
    const int k = i & 1;
    const float hx = c->p.plat_half_x, hz = c->p.plat_half_z;
    float nx, nz, tx, tz;
    switch (j) {
        case 0: nx = 1.0f;  nz = 0.0f;  tx = 0.0f; tz = 1.0f; break;
        case 1: nx = 0.0f;  nz = 1.0f;  tx = -1.0f; tz = 0.0f; break;
        case 2: nx = -1.0f; nz = 0.0f;  tx = 0.0f; tz = -1.0f; break;
        default: nx = 0.0f; nz = -1.0f; tx = 1.0f; tz = 0.0f; break;
    }
#if CDPR_TOPOLOGY_CORNER
    /* 严格 8 角：系绳点在平台 4 个角上，上绳接上表面、下绳接下表面。
     * 与 core.js 的 corners 一致：
     *   j=0 → (+1,+1)  j=1 → (+1,-1)  j=2 → (-1,-1)  j=3 → (-1,+1) */
    float px, pz;
    switch (j) {
        case 0: px = 1.0f;  pz = 1.0f;  break;
        case 1: px = 1.0f;  pz = -1.0f; break;
        case 2: px = -1.0f; pz = -1.0f; break;
        default: px = -1.0f; pz = 1.0f; break;
    }
    out[0] = px * hx;
    out[1] = (k == 0) ? c->p.plat_thick * 0.5f : -c->p.plat_thick * 0.5f;
    out[2] = pz * hz;
#else
    float sx, sz;
    if (k == 0) {
        /* 该边第一个角点（上表面） */
        switch (j) {
            case 0: sx = 1.0f;  sz = 1.0f;  break;
            case 1: sx = 1.0f;  sz = 1.0f;  break;
            case 2: sx = -1.0f; sz = 1.0f;  break;
            default: sx = 1.0f; sz = -1.0f; break;
        }
    } else {
        /* 该边第二个角点（下表面） */
        switch (j) {
            case 0: sx = 1.0f;  sz = -1.0f; break;
            case 1: sx = -1.0f; sz = 1.0f;  break;
            case 2: sx = -1.0f; sz = -1.0f; break;
            default: sx = -1.0f; sz = -1.0f; break;
        }
    }
    out[0] = sx * hx;
    out[1] = (k == 0) ? c->p.plat_thick * 0.5f : -c->p.plat_thick * 0.5f;
    out[2] = sz * hz;
#endif
}

/* 说明：旧版曾用「下绳接对角角点」来强行让结构矩阵满秩，
 * 那会让绳索在俯视图上交叉。现改为「四边中点锚点 + 同边双角点」，
 * 既满秩又不交叉（与 sim/core/core.js 的 buildModel 一致）。 */

/* R = Rz(yaw) * Ry(pitch) * Rx(roll) */
static void rpy_to_r(const float rpy[3], float R[3][3]) {
    const float cr = cosf(rpy[0]), sr = sinf(rpy[0]);
    const float cp = cosf(rpy[1]), sp = sinf(rpy[1]);
    const float cy = cosf(rpy[2]), sy = sinf(rpy[2]);
    R[0][0] = cy * cp; R[0][1] = cy * sp * sr - sy * cr; R[0][2] = cy * sp * cr + sy * sr;
    R[1][0] = sy * cp; R[1][1] = sy * sp * sr + cy * cr; R[1][2] = sy * sp * cr - cy * sr;
    R[2][0] = -sp;     R[2][1] = cp * sr;                R[2][2] = cp * cr;
}

void cdpr_geometry(const cdpr_ctl_t *c, const float x[CDPR_DOF], cdpr_geom_t *out) {
    float R[3][3];
    rpy_to_r(&x[3], R);
    for (int i = 0; i < CDPR_MAX_CABLES; i++) {
        float b[3], A[3];
        point_of(c, i, b);
        anchor_of(c, i, A);
        /* 平台点世界坐标 */
        float B[3];
        for (int k = 0; k < 3; k++) {
            B[k] = x[k] + R[k][0] * b[0] + R[k][1] * b[1] + R[k][2] * b[2];
        }
        out->point[i][0] = B[0];
        out->point[i][1] = B[1];
        out->point[i][2] = B[2];
        /* 绳向量 u = A - B，长度与单位方向 */
        float u[3];
        u[0] = A[0] - B[0];
        u[1] = A[1] - B[1];
        u[2] = A[2] - B[2];
        float len = sqrtf(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
        if (len < 1e-6f) len = 1e-6f;
        out->length[i] = len;
        const float sx = u[0] / len, sy = u[1] / len, sz = u[2] / len;
        out->dir[i][0] = sx;
        out->dir[i][1] = sy;
        out->dir[i][2] = sz;
        /* 力矩臂 d = B - p */
        const float dx = B[0] - x[0], dy = B[1] - x[1], dz = B[2] - x[2];
        out->A[0][i] = sx;
        out->A[1][i] = sy;
        out->A[2][i] = sz;
        out->A[3][i] = dy * sz - dz * sy;
        out->A[4][i] = dz * sx - dx * sz;
        out->A[5][i] = dx * sy - dy * sx;
    }
}

float cdpr_cable_stiffness(const cdpr_params_t *p, float len) {
    const float area = PI_F * (p->cable_diameter * 0.5f) * (p->cable_diameter * 0.5f);
    const float l = len < 0.02f ? 0.02f : len;
    return p->cable_emodulus * area / l;
}

void cdpr_external_wrench(const cdpr_params_t *p, const float contact_f[3], float w[CDPR_DOF]) {
    /* 锚点高于平台时，锁定平衡要求 w = +m g e_y - F_contact */
    w[0] = -contact_f[0];
    w[1] = p->plat_mass * p->gravity - contact_f[1];
    w[2] = -contact_f[2];
    w[3] = 0.0f;
    w[4] = 0.0f;
    w[5] = 0.0f;
}

/* 6x6 线性方程组：高斯消元（列主元） */
static int solve6(float M[6][6], float b[6], float x[6]) {
    float a[6][7];
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) a[i][j] = M[i][j];
        a[i][6] = b[i];
    }
    for (int col = 0; col < 6; col++) {
        int piv = col;
        for (int r = col + 1; r < 6; r++) {
            if (fabsf(a[r][col]) > fabsf(a[piv][col])) piv = r;
        }
        if (fabsf(a[piv][col]) < 1e-12f) return -1;
        if (piv != col) {
            for (int k = 0; k < 7; k++) {
                const float t = a[col][k];
                a[col][k] = a[piv][k];
                a[piv][k] = t;
            }
        }
        const float d = a[col][col];
        for (int r = col + 1; r < 6; r++) {
            const float f = a[r][col] / d;
            if (f == 0.0f) continue;
            for (int k = col; k < 7; k++) a[r][k] -= f * a[col][k];
        }
    }
    for (int r = 5; r >= 0; r--) {
        float s = a[r][6];
        for (int k = r + 1; k < 6; k++) s -= a[r][k] * x[k];
        x[r] = s / a[r][r];
    }
    return 0;
}

/* 非负最小二乘（投影梯度）：min ||A T - w||^2 s.t. T >= 0。
 * 与 sim/core/core.js 的 nonNegativeSolve（mode 2）一致：这是物理上唯一
 * 正确的「绳只能拉」张力分配，最小范数解会给出负张力。
 * 用上次的解做热启动（t 传入初值），1 kHz 下每周期只需几十次迭代即可收敛，
 * 也避免了「每周期从 T=1 重新算」导致的张力抖动。 */
static void nnls_solve(const cdpr_geom_t *g, const float w[CDPR_DOF],
                       float t[CDPR_MAX_CABLES], int iters) {
    float L = 0.0f;
    for (int i = 0; i < 6; i++) {
        float s = 0.0f;
        for (int j = 0; j < CDPR_MAX_CABLES; j++) s += fabsf(g->A[i][j]);
        if (s > L) L = s;
    }
    const float lam = 1.0f / (L * L > 1e-9f ? L * L : 1e-9f);
    for (int it = 0; it < iters; it++) {
        float r[6];
        for (int k = 0; k < 6; k++) {
            float s = w[k];
            for (int i = 0; i < CDPR_MAX_CABLES; i++) s -= g->A[k][i] * t[i];
            r[k] = s;
        }
        float move = 0.0f;
        for (int i = 0; i < CDPR_MAX_CABLES; i++) {
            float gr = 0.0f;
            for (int k = 0; k < 6; k++) gr += g->A[k][i] * r[k];
            const float v = t[i] + lam * gr;
            const float nv = v > 0.0f ? v : 0.0f;
            const float d = fabsf(nv - t[i]);
            if (d > move) move = d;
            t[i] = nv;
        }
        if (move < 1e-4f) break;
    }
}

/* 在 A 的零空间里找「尽可能正」的方向 z（与 JS 的 findPositiveNullspace 一致）。
 * 做法：Gram-Schmidt 求 A 行空间正交基 → 投影矩阵 P = I - Σ b b^T，
 * 再对 z 做「最大化最小分量」的梯度上升，z 始终保持在零空间内。
 * 返回 1 表示找到全正方向，0 表示该位姿无法力封闭。 */
static int positive_nullspace(const cdpr_geom_t *g, float z[CDPR_MAX_CABLES]) {
    const int n = CDPR_MAX_CABLES;
    float basis[6][CDPR_MAX_CABLES];
    int nb = 0;
    for (int k = 0; k < 6; k++) {
        float v[CDPR_MAX_CABLES];
        for (int i = 0; i < n; i++) v[i] = g->A[k][i];
        for (int b = 0; b < nb; b++) {
            float d = 0.0f;
            for (int i = 0; i < n; i++) d += basis[b][i] * v[i];
            for (int i = 0; i < n; i++) v[i] -= d * basis[b][i];
        }
        float nv = 0.0f;
        for (int i = 0; i < n; i++) nv += v[i] * v[i];
        nv = sqrtf(nv);
        if (nv > 1e-6f) {
            for (int i = 0; i < n; i++) basis[nb][i] = v[i] / nv;
            nb++;
        }
    }

    /* 投影到零空间 */
    float zz[CDPR_MAX_CABLES];
    const float inv = 1.0f / sqrtf((float)n);
    for (int i = 0; i < n; i++) zz[i] = inv;
    for (int b = 0; b < nb; b++) {
        float d = 0.0f;
        for (int i = 0; i < n; i++) d += basis[b][i] * zz[i];
        for (int i = 0; i < n; i++) zz[i] -= d * basis[b][i];
    }
    float nz = 0.0f;
    for (int i = 0; i < n; i++) nz += zz[i] * zz[i];
    nz = sqrtf(nz);
    if (nz < 1e-6f) return 0;
    for (int i = 0; i < n; i++) zz[i] /= nz;

    float best = 1e9f;
    for (int i = 0; i < n; i++) if (zz[i] < best) best = zz[i];
    float bestZ[CDPR_MAX_CABLES];
    for (int i = 0; i < n; i++) bestZ[i] = zz[i];

    for (int it = 0; it < 4000; it++) {
        int minI = 0;
        for (int i = 1; i < n; i++) if (zz[i] < zz[minI]) minI = i;
        /* 朝「提升最小分量」的方向走，再投回零空间 */
        float gr[CDPR_MAX_CABLES];
        for (int i = 0; i < n; i++) gr[i] = 0.0f;
        gr[minI] = 1.0f;
        for (int b = 0; b < nb; b++) {
            float d = 0.0f;
            for (int i = 0; i < n; i++) d += basis[b][i] * gr[i];
            for (int i = 0; i < n; i++) gr[i] -= d * basis[b][i];
        }
        const float step = 0.08f;
        float nn = 0.0f;
        for (int i = 0; i < n; i++) { zz[i] += step * gr[i]; nn += zz[i] * zz[i]; }
        nn = sqrtf(nn);
        if (nn < 1e-9f) break;
        for (int i = 0; i < n; i++) zz[i] /= nn;
        float m = 1e9f;
        for (int i = 0; i < n; i++) if (zz[i] < m) m = zz[i];
        if (m > best) {
            best = m;
            for (int i = 0; i < n; i++) bestZ[i] = zz[i];
        }
    }
    if (best <= 1e-4f) return 0;
    for (int i = 0; i < n; i++) z[i] = bestZ[i];
    return 1;
}

/* 预紧：把张力沿零空间抬到 t_min 以上，不改变 A·T（残差仍为 0）。 */
static void apply_preload(const cdpr_geom_t *g, float t[CDPR_MAX_CABLES], float tmin) {
    const int n = CDPR_MAX_CABLES;
    int need = 0;
    for (int i = 0; i < n; i++) if (t[i] < tmin) { need = 1; break; }
    if (!need) return;

    float z[CDPR_MAX_CABLES];
    if (!positive_nullspace(g, z)) return;

    float alpha = -1e9f;
    for (int i = 0; i < n; i++) {
        if (z[i] > 1e-6f) {
            const float a = (tmin - t[i]) / z[i];
            if (a > alpha) alpha = a;
        }
    }
    if (alpha <= 0.0f || alpha > 1e8f) return;

    /* 校验 z 确实在零空间里（避免误伤别的拓扑） */
    float err = 0.0f;
    for (int k = 0; k < 6; k++) {
        float s = 0.0f;
        for (int i = 0; i < n; i++) s += g->A[k][i] * z[i];
        if (fabsf(s) > err) err = fabsf(s);
    }
    if (err > 1e-4f) return;
    for (int i = 0; i < n; i++) t[i] += alpha * z[i];
}

int cdpr_allocate(const cdpr_ctl_t *c, const float x[CDPR_DOF], const float w[CDPR_DOF],
                  float t_out[CDPR_MAX_CABLES]) {
    cdpr_geom_t g;
    cdpr_geometry(c, x, &g);
    /* 非负最小二乘（与 JS mode 2 一致）：这是「绳只能拉」的物理正确解。
     * 用传入的 t_out 做热启动；1 kHz 控制下几轮就能跟踪，
     * 但单次调用（如初始化/自检）需要足够迭代才能收敛。 */
    nnls_solve(&g, w, t_out, 20000);
    /* 零空间预紧：NNLS 在角点拓扑下会退化成「只撑一组」，
     * 沿零空间把张力抬到 t_min 以上（与 JS 的 addPreload 一致）。 */
    apply_preload(&g, t_out, c->p.cable_t_min);
    /* 上限幅 */
    float fmin = 1e9f, res = 0.0f;
    for (int i = 0; i < CDPR_MAX_CABLES; i++) {
        if (t_out[i] < 0.0f) t_out[i] = 0.0f;
        if (t_out[i] > c->p.cable_t_max) t_out[i] = c->p.cable_t_max;
        if (t_out[i] < fmin) fmin = t_out[i];
    }
    for (int i = 0; i < 6; i++) {
        float s = w[i];
        for (int k = 0; k < CDPR_MAX_CABLES; k++) s -= g.A[i][k] * t_out[k];
        if (fabsf(s) > res) res = fabsf(s);
    }
    return (fmin > 0.0f && res < 1.0f) ? 0 : -1;
}

void cdpr_fk(const cdpr_ctl_t *c, const float len[CDPR_MAX_CABLES], float x_inout[CDPR_DOF], int iters) {
    cdpr_geom_t g;
    for (int it = 0; it < iters; it++) {
        cdpr_geometry(c, x_inout, &g);
        /* 残差 e = len - l(x) */
        float e[CDPR_MAX_CABLES];
        for (int i = 0; i < CDPR_MAX_CABLES; i++) e[i] = len[i] - g.length[i];
        /* 雅可比 J = -A^T（8x6），用最小二乘 dx = J^+ e */
        float JT[CDPR_MAX_CABLES][6];
        for (int i = 0; i < CDPR_MAX_CABLES; i++) {
            for (int k = 0; k < 6; k++) JT[i][k] = -g.A[k][i];
        }
        float H[6][6];
        float b[6];
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                float s = 0.0f;
                for (int k = 0; k < CDPR_MAX_CABLES; k++) s += JT[k][i] * JT[k][j];
                H[i][j] = s;
            }
            H[i][i] += 1e-6f;
            float s = 0.0f;
            for (int k = 0; k < CDPR_MAX_CABLES; k++) s += JT[k][i] * e[k];
            b[i] = s;
        }
        float dx[6];
        if (solve6(H, b, dx) != 0) return;
        /* 限步长 */
        float big = 0.0f;
        for (int k = 0; k < 6; k++) if (fabsf(dx[k]) > big) big = fabsf(dx[k]);
        float step = 1.0f;
        if (big > 0.1f) step = 0.1f / big;
        for (int k = 0; k < 6; k++) x_inout[k] += step * dx[k];
    }
}

void cdpr_init(cdpr_ctl_t *c, const cdpr_params_t *p) {
    memset(c, 0, sizeof(*c));
    c->p = *p;
    for (int k = 0; k < CDPR_DOF; k++) {
        c->q[k] = p->home_pose[k];
        c->q_target[k] = p->home_pose[k];
        c->prevQ[k] = p->home_pose[k];
    }
    cdpr_geometry(c, c->q, &c->g);
    float w[CDPR_DOF];
    cdpr_external_wrench(&c->p, (float[3]){0, 0, 0}, w);
    /* 初值必须显式给（不能是未初始化的栈内存），否则 NNLS 热启动会从垃圾值出发 */
    float t[CDPR_MAX_CABLES];
    for (int i = 0; i < CDPR_MAX_CABLES; i++) t[i] = p->cable_t_min;
    cdpr_allocate(c, c->q, w, t);
    for (int i = 0; i < CDPR_MAX_CABLES; i++) {
        c->tension_ref[i] = t[i];
        c->tension_filt[i] = t[i];
        c->len_ref[i] = c->g.length[i] - t[i] / cdpr_cable_stiffness(&c->p, c->g.length[i]);
        c->len_prev[i] = c->len_ref[i];
    }
    c->feasible = 1;
}

/* 一维二阶限速：不超过 vmax、变化率不超过 amax */
static float rate_limit(float desired, float prev, float *vel, float dt, float vmax, float amax) {
    const float err = desired - prev;
    float v = *vel;
    if ((err > 0.0f) != (v > 0.0f)) v = 0.0f;
    float vdes = (err >= 0.0f ? 1.0f : -1.0f) * vmax;
    const float vCap = sqrtf(2.0f * amax * fabsf(err));
    if (vCap < fabsf(vdes)) vdes = (err >= 0.0f ? 1.0f : -1.0f) * vCap;
    const float dv = vdes - v;
    const float dvMax = amax * dt;
    v += (dv > dvMax) ? dvMax : ((dv < -dvMax) ? -dvMax : dv);
    float next = prev + v * dt;
    if ((desired - prev) * (desired - next) < 0.0f) {
        next = desired;
        v = 0.0f;
    }
    *vel = v;
    return next;
}

void cdpr_control_step(cdpr_ctl_t *c, const float q_target[CDPR_DOF],
                       const float meas_t[CDPR_MAX_CABLES], int contact, float dt,
                       cdpr_cmd_t cmd_out[CDPR_MAX_CABLES]) {
    for (int k = 0; k < CDPR_DOF; k++) c->q_target[k] = q_target[k];
    cdpr_geom_t g;
    cdpr_geometry(c, q_target, &g);

    /* 外力：重力 + 地面支撑 */
    float contactF[3] = {0.0f, 0.0f, 0.0f};
    if (contact) contactF[1] = -c->p.plat_mass * c->p.gravity;
    float w[CDPR_DOF];
    cdpr_external_wrench(&c->p, contactF, w);

    /* 加速度前馈：目标位姿的二阶差分（低通） */
    if (c->have_prev) {
        float v[CDPR_DOF], acc[CDPR_DOF];
        for (int k = 0; k < CDPR_DOF; k++) v[k] = (q_target[k] - c->prevQ[k]) / dt;
        for (int k = 0; k < CDPR_DOF; k++) acc[k] = (v[k] - c->vFilt[k]) / dt;
        for (int k = 0; k < CDPR_DOF; k++) c->vFilt[k] = v[k];
        const float mass = c->p.plat_mass;
        const float Ix = mass * (0.15f * 0.15f) / 12.0f;
        w[0] += mass * acc[0];
        w[1] += mass * acc[1];
        w[2] += mass * acc[2];
        w[3] += Ix * acc[3];
        w[4] += Ix * acc[4];
        w[5] += Ix * acc[5];
        (void)c->aFilt;
    } else {
        c->have_prev = 1;
    }
    for (int k = 0; k < CDPR_DOF; k++) c->prevQ[k] = q_target[k];

    /* 张力分配 + 低通（用上一周期的张力做 NNLS 热启动，避免每周期抖动） */
    float t[CDPR_MAX_CABLES];
    for (int i = 0; i < CDPR_MAX_CABLES; i++) t[i] = c->tension_ref[i];
    const int ok = cdpr_allocate(c, q_target, w, t);
    c->feasible = (ok == 0);
    const float a = 1.0f - expf(-2.0f * PI_F * 10.0f * dt);
    for (int i = 0; i < CDPR_MAX_CABLES; i++) c->tension_ref[i] += (t[i] - c->tension_ref[i]) * a;

    /* 绳长指令：弹性预补偿 + 张力闭环 + 限速 */
    const float af = 1.0f - expf(-2.0f * PI_F * 25.0f * dt);
    for (int i = 0; i < CDPR_MAX_CABLES; i++) {
        const float mt = (meas_t != 0) ? meas_t[i] : c->tension_filt[i];
        c->tension_filt[i] += (mt - c->tension_filt[i]) * af;
        const float k = cdpr_cable_stiffness(&c->p, g.length[i]);
        const float dT = c->tension_ref[i] - c->tension_filt[i];
        float dL = -c->p.tension_kp * dT;
        if (dL > 2e-3f) dL = 2e-3f;
        if (dL < -2e-3f) dL = -2e-3f;
        const float desired = g.length[i] - c->tension_ref[i] / k + dL;
        c->len_ref[i] = rate_limit(desired, c->len_prev[i], &c->len_vel[i], dt,
                                   c->p.lref_vmax, c->p.lref_amax);
        c->len_prev[i] = c->len_ref[i];
        if (cmd_out) {
            cmd_out[i].len_target = c->len_ref[i];
            cmd_out[i].motor_trq = -(c->tension_ref[i] * c->p.drum_radius) / c->p.gear_ratio;
        }
    }
    c->cycles++;
}
