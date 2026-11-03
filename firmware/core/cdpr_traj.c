/**
 * 轨迹生成实现（C）
 */
#include "cdpr_traj.h"
#include <math.h>
#include <string.h>

#define PI_F 3.14159265358979f

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* 五次多项式（0->1 平滑，起止速度/加速度为零） */
static float smooth5(float s) {
    s = clampf(s, 0.0f, 1.0f);
    return s * s * s * (10.0f - 15.0f * s + 6.0f * s * s);
}

void traj_init(cdpr_traj_t *t, const cdpr_params_t *p) {
    memset(t, 0, sizeof(*t));
    t->mode = TRAJ_HOLD;
    for (int k = 0; k < CDPR_DOF; k++) t->q[k] = p->home_pose[k];
    t->period = 10.0f;
    t->xy_amp = 0.12f;
    t->z_amp = 0.05f;
    t->height = p->home_pose[1];
    t->roll_amp = 0.0f;
    t->pitch_amp = 0.08f;
    t->yaw_amp = 0.12f;
    t->pallet_seg_time = 1.5f;
    t->pallet_acc = 0.15f;
}

void traj_goto(cdpr_traj_t *t, const float target[CDPR_DOF], float dur) {
    for (int k = 0; k < CDPR_DOF; k++) {
        t->q0[k] = t->q[k];
        t->q1[k] = target[k];
    }
    t->t = 0.0f;
    t->duration = dur < 0.3f ? 0.3f : dur;
    t->mode = TRAJ_LINE;
}

void traj_circle(cdpr_traj_t *t) {
    for (int k = 0; k < CDPR_DOF; k++) t->q0[k] = t->q[k];
    t->t = 0.0f;
    t->mode = TRAJ_CIRCLE;
}

void traj_hold(cdpr_traj_t *t) {
    for (int k = 0; k < CDPR_DOF; k++) t->q0[k] = t->q[k];
    t->mode = TRAJ_HOLD;
}

void traj_pallet(cdpr_traj_t *t, const cdpr_params_t *p) {
    /* 十字取放：4 行，每行「取料 -> 抬升 -> 转运 -> 放料」 */
    const int n = 4;
    const float spacing = 0.15f;
    const float pick_x = -0.20f;
    const float pick_h = 0.41f;
    const float lift_h = 0.52f;
    const float place_x = (spacing * (n - 1)) * 0.5f;
    int idx = 0;
    for (int i = 0; i < n; i++) {
        const float z = (i - (n - 1) * 0.5f) * spacing;
        t->pallet_pts[idx][0] = pick_x;  t->pallet_pts[idx][1] = pick_h; t->pallet_pts[idx][2] = z; idx++;
        t->pallet_pts[idx][0] = pick_x;  t->pallet_pts[idx][1] = lift_h; t->pallet_pts[idx][2] = z; idx++;
        t->pallet_pts[idx][0] = place_x; t->pallet_pts[idx][1] = lift_h; t->pallet_pts[idx][2] = z; idx++;
        t->pallet_pts[idx][0] = place_x; t->pallet_pts[idx][1] = pick_h; t->pallet_pts[idx][2] = z; idx++;
        t->pallet_pts[idx][0] = place_x; t->pallet_pts[idx][1] = lift_h; t->pallet_pts[idx][2] = z; idx++;
        t->pallet_pts[idx][0] = pick_x;  t->pallet_pts[idx][1] = lift_h; t->pallet_pts[idx][2] = z; idx++;
    }
    t->pallet_n = idx;
    t->pallet_idx = 0;
    t->pallet_t = 0.0f;
    t->mode = TRAJ_PALLET;
    (void)p;
}

void traj_step(cdpr_traj_t *t, const cdpr_params_t *p, float dt, float q_out[CDPR_DOF]) {
    t->t += dt;
    for (int k = 0; k < CDPR_DOF; k++) q_out[k] = t->q[k];
    switch (t->mode) {
        case TRAJ_LINE: {
            const float s = smooth5(t->t / t->duration);
            for (int k = 0; k < CDPR_DOF; k++) q_out[k] = t->q0[k] + (t->q1[k] - t->q0[k]) * s;
            if (t->t >= t->duration) {
                for (int k = 0; k < CDPR_DOF; k++) t->q[k] = t->q1[k];
                t->mode = TRAJ_HOLD;
            }
            break;
        }
        case TRAJ_CIRCLE: {
            const float w = 2.0f * PI_F / (t->period > 0.5f ? t->period : 0.5f);
            const float ph = w * t->t;
            const float blend = smooth5(t->t / 3.0f);
            const float target[CDPR_DOF] = {
                t->xy_amp * sinf(ph),
                t->height + t->z_amp * sinf(2.0f * ph),
                t->z_amp * 0.6f * sinf(ph),
                t->roll_amp * sinf(ph),
                t->pitch_amp * cosf(ph),
                t->yaw_amp * sinf(0.5f * ph),
            };
            for (int k = 0; k < CDPR_DOF; k++) q_out[k] = t->q0[k] + (target[k] - t->q0[k]) * blend;
            break;
        }
        case TRAJ_PALLET: {
            if (t->pallet_idx >= t->pallet_n - 1) {
                t->mode = TRAJ_HOLD;
                break;
            }
            t->pallet_t += dt;
            const float f = smooth5(t->pallet_t / t->pallet_seg_time);
            const float *A = t->pallet_pts[t->pallet_idx];
            const float *B = t->pallet_pts[t->pallet_idx + 1];
            q_out[0] = A[0] + (B[0] - A[0]) * f;
            q_out[1] = A[1] + (B[1] - A[1]) * f;
            q_out[2] = A[2] + (B[2] - A[2]) * f;
            q_out[3] = 0.0f; q_out[4] = 0.0f; q_out[5] = 0.0f;
            if (t->pallet_t >= t->pallet_seg_time) {
                t->pallet_idx++;
                t->pallet_t = 0.0f;
            }
            break;
        }
        default:
            break;
    }
    /* 工作空间限幅 */
    const float mx = p->plat_half_x + 0.06f;
    const float mz = p->plat_half_z + 0.06f;
    q_out[0] = clampf(q_out[0], -mx, mx);
    q_out[2] = clampf(q_out[2], -mz, mz);
    q_out[1] = clampf(q_out[1], p->floor_y + 0.03f, p->anchor_lower_y - 0.05f);
    q_out[3] = clampf(q_out[3], -0.45f, 0.45f);
    q_out[4] = clampf(q_out[4], -0.45f, 0.45f);
    q_out[5] = clampf(q_out[5], -3.1f, 3.1f);
    for (int k = 0; k < CDPR_DOF; k++) t->q[k] = q_out[k];
}
