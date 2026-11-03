/**
 * 轨迹生成（C，与 sim/core/trajectory.js 对应）
 */
#ifndef CDPR_TRAJ_H
#define CDPR_TRAJ_H

#include "cdpr_core.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TRAJ_HOLD = 0,
    TRAJ_LINE = 1,
    TRAJ_CIRCLE = 2,
    TRAJ_PALLET = 3
} cdpr_traj_mode_t;

typedef struct {
    cdpr_traj_mode_t mode;
    float q[CDPR_DOF];      /* 当前输出位姿 */
    float q0[CDPR_DOF];     /* 段起点 */
    float q1[CDPR_DOF];     /* 段终点 */
    float t;                /* 段内时间 */
    float duration;         /* 段时长 */
    float period;           /* 圆轨迹周期 */
    float xy_amp, z_amp;    /* 圆轨迹幅度 */
    float roll_amp, pitch_amp, yaw_amp;
    float height;
    /* 码垛 */
    float pallet_pts[64][3];
    int   pallet_n;
    float pallet_seg_time;
    float pallet_acc;
    int   pallet_idx;
    float pallet_t;
} cdpr_traj_t;

void traj_init(cdpr_traj_t *t, const cdpr_params_t *p);
void traj_goto(cdpr_traj_t *t, const float target[CDPR_DOF], float dur);
void traj_circle(cdpr_traj_t *t);
void traj_pallet(cdpr_traj_t *t, const cdpr_params_t *p);
void traj_hold(cdpr_traj_t *t);
/** 推进 dt，输出目标位姿（带限幅） */
void traj_step(cdpr_traj_t *t, const cdpr_params_t *p, float dt, float q_out[CDPR_DOF]);

#ifdef __cplusplus
}
#endif

#endif
