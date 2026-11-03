/**
 * STM32F407ZGT6 移植层接口
 *
 * 与 control_main.h 中的 ctl_hal_t 对接：把板级读写在 ctl_tick 前后调用。
 */
#ifndef STM32F407_PORT_H
#define STM32F407_PORT_H

#include "../core/cdpr_core.h"

#ifdef __cplusplus
extern "C" {
#endif

void cdpr_port_init(const cdpr_params_t *p);
void cdpr_port_set_steps(int motor, int steps);
void cdpr_port_read_cable_pos(float out_m[CDPR_MAX_CABLES]);
void cdpr_port_read_tension(float out_n[CDPR_MAX_CABLES]);
int cdpr_port_read_contact(void);

#ifdef __cplusplus
}
#endif

#endif
