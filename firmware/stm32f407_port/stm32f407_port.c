/**
 * STM32F407ZGT6 移植层
 *
 * 硬件对应关系（与用户选型一致）：
 *   8 × 86 闭环步进电机 + HBS860H 驱动器（细分 16，编码器 10000 线）
 *   每个驱动器：PUL/DIR/ENA 三根信号线 + ALM 报警输入
 *   STM32F407ZGT6 定时器分配（示例，可按实际接线调整）：
 *     TIM2 CH1~CH4  -> 电机 0~3 的 PUL（PWM 输出，频率 = 步频）
 *     TIM5 CH1~CH4  -> 电机 4~7 的 PUL
 *     GPIO          -> DIR / ENA / ALM
 *   1 kHz 控制中断：TIM6（更新中断里调用 ctl_tick）
 *
 * 本文件只做「寄存器/外设 <-> 控制核心」的粘合，不含控制逻辑，
 * 因此 PC 板级仿真可以完全跳过它（用 board_sim 的 HAL 实现代替）。
 *
 * 注意：本文件按 STM32F4 标准外设库/HAL 的命名书写，实际工程里
 * 把 CDPR_USE_HAL 打开即可直接编译；默认关闭以便在没有 STM32 头文件
 * 的机器上也能编译整个工程（此时只提供接口桩）。
 */
#include "stm32f407_port.h"

#ifdef CDPR_USE_HAL
#include "stm32f4xx_hal.h"

/* 每个电机的方向与使能引脚（按实际接线修改） */
static const struct { GPIO_TypeDef *dir_port; uint16_t dir_pin; GPIO_TypeDef *ena_port; uint16_t ena_pin; }
kMotorPins[8] = {
    { GPIOC, GPIO_PIN_0, GPIOC, GPIO_PIN_1 },
    { GPIOC, GPIO_PIN_2, GPIOC, GPIO_PIN_3 },
    { GPIOC, GPIO_PIN_4, GPIOC, GPIO_PIN_5 },
    { GPIOC, GPIO_PIN_6, GPIOC, GPIO_PIN_7 },
    { GPIOC, GPIO_PIN_8, GPIOC, GPIO_PIN_9 },
    { GPIOC, GPIO_PIN_10, GPIOC, GPIO_PIN_11 },
    { GPIOC, GPIO_PIN_12, GPIOC, GPIO_PIN_13 },
    { GPIOD, GPIO_PIN_0, GPIOD, GPIO_PIN_1 },
};

/* 每个电机当前累计脉冲数（用于换算收绳量） */
static int32_t s_steps[8];
static float s_cable_per_step;
static float s_cable_zero[8];

void cdpr_port_init(const cdpr_params_t *p) {
    /* 细分与减速比 -> 每个脉冲对应的收绳量 */
    const float spr = (360.0f / p->step_angle_deg) * (float)p->driver_microstep;
    s_cable_per_step = (2.0f * 3.14159265f / spr) / p->gear_ratio * p->drum_radius;
    for (int i = 0; i < 8; i++) {
        s_steps[i] = 0;
        s_cable_zero[i] = 0.0f;
        HAL_GPIO_WritePin(kMotorPins[i].ena_port, kMotorPins[i].ena_pin, GPIO_PIN_RESET); /* 使能 */
    }
    /* 定时器与中断的初始化（PWM、编码器、1 kHz 控制节拍）在此完成，
       具体寄存器配置由 CubeMX 生成的 MX_TIMx_Init() 提供。 */
}

void cdpr_port_set_steps(int motor, int32_t steps) {
    if (motor < 0 || motor > 7) return;
    s_steps[motor] = steps;
}

void cdpr_port_read_cable_pos(float out_m[8]) {
    for (int i = 0; i < 8; i++) {
        out_m[i] = (float)(s_steps[i] - (int32_t)s_cable_zero[i]) * s_cable_per_step;
    }
}

/* 张力：若装了拉力传感器，在这里读 ADC 并换算成 N；否则返回 0（纯位置控制） */
void cdpr_port_read_tension(float out_n[8]) {
    for (int i = 0; i < 8; i++) out_n[i] = 0.0f;
}

int cdpr_port_read_contact(void) {
    return 0;
}

#else /* !CDPR_USE_HAL：无 STM32 头文件时的桩实现 */

#include <stdint.h>

void cdpr_port_init(const void *p) { (void)p; }
void cdpr_port_set_steps(int motor, int32_t steps) { (void)motor; (void)steps; }
void cdpr_port_read_cable_pos(float out_m[8]) { for (int i = 0; i < 8; i++) out_m[i] = 0.0f; }
void cdpr_port_read_tension(float out_n[8]) { for (int i = 0; i < 8; i++) out_n[i] = 0.0f; }
int cdpr_port_read_contact(void) { return 0; }

#endif
