/**
 * 控制主程序（平台无关，STM32 与 PC 共用）
 *
 * 结构：
 *   ctl_boot_t   板级资源（定时器、脉冲输出、编码器读数、张力 ADC）
 *   ctl_init()   上电初始化：参数、控制器、轨迹
 *   ctl_tick()   一个 1 kHz 控制周期：读编码器 -> 正运动学 -> 轨迹 -> 控制 -> 输出
 *
 * 在 STM32 上，ctl_tick() 由 TIM 中断调用；在 PC 板级仿真中由 board_sim 调用。
 * 定时器/外设相关代码放在 firmware/stm32f407_port/，本文件不直接碰寄存器。
 */
#ifndef CONTROL_MAIN_H
#define CONTROL_MAIN_H

#include "../core/cdpr_core.h"
#include "../core/cdpr_traj.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CTL_RATE_HZ 1000
#define CTL_DT (1.0f / (float)CTL_RATE_HZ)

/** 板级接口：由 stm32f407_port 或 board_sim 实现 */
typedef struct {
    /** 读取 8 个卷筒的累计收绳量（m），来自 HBS860H 编码器计数 */
    void (*read_cable_pos)(float out_m[CDPR_MAX_CABLES]);
    /** 读取 8 根绳的张力（N）；没有传感器时置 0 */
    void (*read_tension)(float out_n[CDPR_MAX_CABLES]);
    /** 输出 8 根绳的目标绳长（m）与力矩前馈（Nm） */
    void (*write_cmd)(const float len_target[CDPR_MAX_CABLES], const float trq[CDPR_MAX_CABLES]);
    /** 读取平台是否接触地面（限位/称重） */
    int (*read_contact)(void);
} ctl_hal_t;

typedef struct {
    cdpr_ctl_t ctl;
    cdpr_traj_t traj;
    ctl_hal_t *hal;
    cdpr_params_t params;
    float cable_pos[CDPR_MAX_CABLES];
    float tension[CDPR_MAX_CABLES];
    float len_abs[CDPR_MAX_CABLES];
    float q_meas[CDPR_DOF];
    float sim_time;
    unsigned long tick;
    int contact;
} ctl_boot_t;

void ctl_init(ctl_boot_t *b, cdpr_ctl_t *ctl, cdpr_traj_t *traj, const cdpr_params_t *p);
void ctl_set_hal(ctl_boot_t *b, ctl_hal_t *hal);
void ctl_tick(ctl_boot_t *b, cdpr_ctl_t *ctl, cdpr_traj_t *traj);

#ifdef __cplusplus
}
#endif

#endif
