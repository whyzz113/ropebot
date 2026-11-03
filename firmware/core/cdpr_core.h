/**
 * 绳驱并联机器人控制核心（C 语言，可在 STM32F407 与 PC 板级仿真中共用）
 *
 * 设计目标
 *   - 与 sim/core/*.js 一一对应（同一套参数、同一套公式、同一套张力分配）
 *   - 不依赖任何 HAL / 操作系统，只依赖 libm；STM32 与 PC 都能编译
 *   - 所有状态显式存放在结构体里，便于板级仿真逐周期调用
 *
 * 单位：m、kg、s、N、rad
 */
#ifndef CDPR_CORE_H
#define CDPR_CORE_H

#ifdef __cplusplus
extern "C" {
#endif

#define CDPR_MAX_CABLES 8
#define CDPR_DOF 6

/* 拓扑开关：由 cdpr_params.h 定义（1 = 角点，0 = 标准四边中点）。
 * 这里给默认值，保证单独 include cdpr_core.h 时也能编译；
 * cdpr_params.h 会覆盖它（见该文件里的 #undef / #define）。 */
#ifndef CDPR_TOPOLOGY_CORNER
#define CDPR_TOPOLOGY_CORNER 0
#endif

/** 参数：由 JSON 转换生成的 cdpr_params.h 填充（见 tools/gen_params.py） */
typedef struct {
    /* 框架 */
    float frame_width, frame_depth, frame_top_y;
    /* 动平台 */
    float plat_half_x, plat_half_z, plat_thick, plat_mass, gravity;
    /* 卷筒 / 锚点 */
    float anchor_x, anchor_z, anchor_upper_y, anchor_lower_y, drum_radius;
    float edge_offset;   /* 同边上下两锚点沿边的微错开量（m） */
    /* 角点拓扑（topology = "corner" 时使用）：
     *   锚点在框架四角（法向 corner_spacing），上下绳沿边错开 corner_offset；
     *   系绳点在平台四边中点，上下绳沿边错开 tether_offset。 */
    float corner_spacing, corner_offset, tether_offset;
    /* 绳索 */
    float cable_emodulus, cable_diameter, cable_t_min, cable_t_max;
    /* 电机 / 驱动器 */
    float hold_torque, step_angle_deg, gear_ratio, gear_efficiency;
    int driver_microstep;
    /* 控制 */
    float max_trans_vel, max_trans_acc, max_ang_acc;
    float kp_alloc;             /* 张力分配权重（预留） */
    float lref_vmax, lref_amax; /* 绳长指令限速 */
    float tension_kp;
    float floor_y;
    /* 初始位姿 */
    float home_pose[CDPR_DOF];
} cdpr_params_t;

/** 绳几何与结构矩阵 */
typedef struct {
    float A[CDPR_DOF][CDPR_MAX_CABLES]; /* 结构矩阵 A = [s ; d x s] */
    float length[CDPR_MAX_CABLES];
    float dir[CDPR_MAX_CABLES][3];      /* 单位方向 s_i（平台 -> 锚点） */
    float point[CDPR_MAX_CABLES][3];    /* 平台系绳点世界坐标 B_i */
} cdpr_geom_t;

/** 控制器状态 */
typedef struct {
    cdpr_params_t p;
    cdpr_geom_t g;
    float q[CDPR_DOF];        /* 当前位姿（由编码器正运动学求得） */
    float q_target[CDPR_DOF]; /* 轨迹目标 */
    float tension_ref[CDPR_MAX_CABLES];
    float tension_filt[CDPR_MAX_CABLES];
    float len_ref[CDPR_MAX_CABLES];
    float len_prev[CDPR_MAX_CABLES];
    float len_vel[CDPR_MAX_CABLES];
    float vFilt[CDPR_DOF];
    float aFilt[CDPR_DOF];
    float prevQ[CDPR_DOF];
    int   have_prev;
    int   feasible;
    float diag_fmin, diag_residual;
    unsigned int cycles;
} cdpr_ctl_t;

/** 每根绳的输出指令（与固件里写定时器/脉冲的逻辑对接） */
typedef struct {
    float len_target; /* 目标绳长 m（绝对） */
    float motor_trq;  /* 电机轴力矩前馈 Nm */
} cdpr_cmd_t;

/* ---- 初始化 ---- */
void cdpr_init(cdpr_ctl_t *c, const cdpr_params_t *p);

/* ---- 几何 ---- */
/** 计算结构矩阵与绳长；x 为位姿 [x y z roll pitch yaw] */
void cdpr_geometry(const cdpr_ctl_t *c, const float x[CDPR_DOF], cdpr_geom_t *out);

/* ---- 张力分配 ----
 * 输入期望等效力 w = A T（锁定平衡时 = 重力 + 外部接触力），
 * 输出 8 根绳张力（最小范数解 + 负值修正）。
 * 返回 0 表示成功，-1 表示不可行。
 */
int cdpr_allocate(const cdpr_ctl_t *c, const float x[CDPR_DOF], const float w[CDPR_DOF],
                  float t_out[CDPR_MAX_CABLES]);

/* ---- 正运动学：由 8 根绳长求位姿（Gauss-Newton，迭代 iters 次） ---- */
void cdpr_fk(const cdpr_ctl_t *c, const float len[CDPR_MAX_CABLES], float x_inout[CDPR_DOF], int iters);

/* ---- 控制周期 ---- */
/**
 * 一个控制周期（默认 1 kHz）。
 * @param q_target 轨迹目标位姿
 * @param meas_t   实测张力（无张力传感器时传 0，仅用前馈）
 * @param contact  平台是否接触地面（1 = 接触）
 * @param cmd_out  输出 8 根绳的指令
 */
void cdpr_control_step(cdpr_ctl_t *c, const float q_target[CDPR_DOF],
                       const float meas_t[CDPR_MAX_CABLES], int contact, float dt,
                       cdpr_cmd_t cmd_out[CDPR_MAX_CABLES]);

/* ---- 辅助 ---- */
float cdpr_cable_stiffness(const cdpr_params_t *p, float len);
void cdpr_external_wrench(const cdpr_params_t *p, const float contact_f[3], float w[CDPR_DOF]);

#ifdef __cplusplus
}
#endif

#endif /* CDPR_CORE_H */
