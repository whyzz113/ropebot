/**
 * 一致性测试：C 核心 vs JS 核心
 *
 * 输出 JSON，由 sim/test/compare_core.mjs 与 JS 侧结果逐项对比。
 * 检查项：
 *   1) 几何（8 根绳长）
 *   2) 结构矩阵（6x8）
 *   3) 张力分配（重力下 8 根绳张力）
 *   4) 正运动学反解精度
 */
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "../core/cdpr_core.h"
#include "../core/cdpr_params.h"

int main(void) {
    cdpr_params_t p = cdpr_default_params();
    cdpr_ctl_t ctl;
    cdpr_init(&ctl, &p);

    /* 直接用参数表里的零位，保证与 JS 侧对比的是同一位姿 */
    const float q[6] = {
        p.home_pose[0], p.home_pose[1], p.home_pose[2],
        p.home_pose[3], p.home_pose[4], p.home_pose[5],
    };
    cdpr_geom_t g;
    cdpr_geometry(&ctl, q, &g);

    printf("{\n");
    printf("  \"length\": [");
    for (int i = 0; i < 8; i++) printf("%s%.6f", i ? "," : "", (double)g.length[i]);
    printf("],\n");

    printf("  \"A\": [\n");
    for (int r = 0; r < 6; r++) {
        printf("    [");
        for (int i = 0; i < 8; i++) printf("%s%.6f", i ? "," : "", (double)g.A[r][i]);
        printf("]%s\n", r < 5 ? "," : "");
    }
    printf("  ],\n");

    float w[6];
    cdpr_external_wrench(&p, (float[3]){0.0f, 0.0f, 0.0f}, w);
    /* 必须显式给初值：cdpr_allocate 用 t 做 NNLS 热启动，
     * 未初始化的栈内存会让迭代从垃圾值出发，收敛到错误解（残差 = 整个重力）。 */
    float t[8];
    for (int i = 0; i < 8; i++) t[i] = p.cable_t_min;
    const int ok = cdpr_allocate(&ctl, q, w, t);
    printf("  \"tension\": [");
    for (int i = 0; i < 8; i++) printf("%s%.6f", i ? "," : "", (double)t[i]);
    printf("],\n");
    printf("  \"allocate_ok\": %d,\n", ok);

    /* 正运动学：从几何绳长反解位姿 */
    float x[6];
    for (int k = 0; k < 6; k++) x[k] = q[k] + 0.02f;
    cdpr_fk(&ctl, g.length, x, 30);
    float err = 0.0f;
    for (int k = 0; k < 6; k++) {
        const float e = fabsf(x[k] - q[k]);
        if (e > err) err = e;
    }
    printf("  \"fk_error\": %.9f,\n", (double)err);
    printf("  \"fk_pose\": [");
    for (int k = 0; k < 6; k++) printf("%s%.6f", k ? "," : "", (double)x[k]);
    printf("]\n");
    printf("}\n");
    return 0;
}
