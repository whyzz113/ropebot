/**
 * 控制主程序实现（平台无关）
 */
#include "control_main.h"
#include <string.h>
#include <math.h>

void ctl_init(ctl_boot_t *b, cdpr_ctl_t *ctl, cdpr_traj_t *traj, const cdpr_params_t *p) {
    memset(b, 0, sizeof(*b));
    b->params = *p;
    cdpr_init(ctl, p);
    traj_init(traj, p);
    b->hal = 0;
    b->sim_time = 0.0f;
    /* 初始绳长 = home 位姿下的几何长度 */
    cdpr_geom_t g;
    cdpr_geometry(ctl, ctl->q, &g);
    for (int i = 0; i < CDPR_MAX_CABLES; i++) {
        b->len_abs[i] = g.length[i];
        b->cable_pos[i] = 0.0f;
    }
    for (int k = 0; k < CDPR_DOF; k++) b->q_meas[k] = ctl->q[k];
}

void ctl_set_hal(ctl_boot_t *b, ctl_hal_t *hal) {
    b->hal = hal;
}

void ctl_tick(ctl_boot_t *b, cdpr_ctl_t *ctl, cdpr_traj_t *traj) {
    /* 1) 读编码器：卷筒收绳量 -> 绝对绳长 */
    if (b->hal && b->hal->read_cable_pos) {
        b->hal->read_cable_pos(b->cable_pos);
    }
    for (int i = 0; i < CDPR_MAX_CABLES; i++) {
        b->len_abs[i] = ctl->g.length[i] - b->cable_pos[i];
    }
    /* 2) 正运动学：由绳长反推位姿（用于状态显示与保护） */
    float q_meas[CDPR_DOF];
    for (int k = 0; k < CDPR_DOF; k++) q_meas[k] = ctl->q[k];
    cdpr_fk(ctl, b->len_abs, q_meas, 4);
    for (int k = 0; k < CDPR_DOF; k++) b->q_meas[k] = q_meas[k];

    /* 3) 轨迹 */
    float q_target[CDPR_DOF];
    traj_step(traj, &b->params, CTL_DT, q_target);

    /* 4) 读张力 */
    if (b->hal && b->hal->read_tension) {
        b->hal->read_tension(b->tension);
    } else {
        for (int i = 0; i < CDPR_MAX_CABLES; i++) b->tension[i] = 0.0f;
    }
    b->contact = (b->hal && b->hal->read_contact) ? b->hal->read_contact() : 0;

    /* 5) 控制 */
    cdpr_cmd_t cmd[CDPR_MAX_CABLES];
    cdpr_control_step(ctl, q_target, b->tension, b->contact, CTL_DT, cmd);

    /* 6) 输出到驱动器（STEP/DIR 脉冲或总线指令） */
    float len_t[CDPR_MAX_CABLES], trq_t[CDPR_MAX_CABLES];
    for (int i = 0; i < CDPR_MAX_CABLES; i++) {
        len_t[i] = cmd[i].len_target;
        trq_t[i] = cmd[i].motor_trq;
    }
    if (b->hal && b->hal->write_cmd) {
        b->hal->write_cmd(len_t, trq_t);
    }

    b->tick++;
    b->sim_time += CTL_DT;
}
