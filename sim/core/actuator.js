/**
 * HBS860H 闭环步进驱动器 + 86 闭环步进电机 + 减速器 + 卷筒 的仿真模型。
 *
 * 真实信号链（与 STM32 固件一一对应）：
 *   STM32 --STEP/DIR 脉冲--> HBS860H --三相电流--> 86 闭环步进电机
 *          |                     |-- 编码器 10000 线位置反馈
 *          |                     |-- 位置环 -> 速度环 PI -> 电流环
 *          |                     |-- 关断 ENA / 堵转 ALM
 *   HBS860H --> 电机轴 --> 减速器（G、效率 eta）--> 卷筒（半径 r）--> 绳索
 *
 * 绳长/收绳量：x = theta_motor / G * r；卷筒侧可用转矩 = T_hold * G * eta。
 * 负载由外部（动平台动力学）以「绳张力折算到电机轴的力矩」回灌，
 * 所以当负荷超出电机能力时，画面里会真实地看到下坠、过冲和失步。
 *
 * 参考（参数含义见 docs/参数表.md）：
 *   HBS860H 手册：STV 速度、STR 转矩、MIC 细分、ENC 编码器
 *   86 闭环步进：保持转矩 12 N·m（配套 86HBS 系列）
 */
import { PI } from './core.js';

export const MODE_POS = 0; // 位置模式（DIR/STEP 脉冲）
export const MODE_VEL = 1; // 速度模式（STV/内部速度，模拟量或串口给定）
export const MODE_TRQ = 2; // 纯转矩模式
export const MODE_TRQ_VEL = 3; // 转矩 + 速度限幅（张力控制常用）

/**
 * @param {object} p 完整参数
 * @param {number} i 绳索引（仅用于标记）
 */
export function createActuator(p, i) {
  const a = p.actuators;
  const r = p.winders.drum_radius;
  const G = a.gear_ratio;
  const spr = (360 / a.step_angle_deg) * a.driver_microstep; // 每转脉冲（电机轴）
  const motorRadPerStep = (2 * PI) / spr;
  const drumRadPerStep = motorRadPerStep / G;
  const cablePerStep = drumRadPerStep * r;
  const eff = a.gear_efficiency;
  const tHold = a.hold_torque * a.limit_current_ratio;

  const st = {
    index: i,
    enabled: false, // HBS860H ENA 使能
    fault: 0, // 0 正常 / 1 堵转 ALM / 2 过流
    stallEvents: 0,
    mode: MODE_POS,
    // 指令
    cmdPosCable: 0, // 卷筒收绳量目标 m（= l0 - l_ref）
    cmdVelCable: 0, // 收绳速度指令 m/s
    cmdTrqMotor: 0, // 电机轴力矩指令 Nm（收绳方向为正）
    // 状态
    steps: 0, // 已接收脉冲（相对上电）
    motorPos: 0, // 电机轴角 rad（编码器）
    motorVel: 0, // rad/s
    motorAcc: 0,
    motorTrq: 0, // 电磁力矩 Nm（收绳方向为正）
    cur: 0, // 相电流 A
    curRef: 0,
    velInt: 0,
    followingErr: 0, // 电机轴 rad：指令 - 实际
    loadTrq: 0,
    drumJLoad: 0,
    cablePos: 0, // 收绳量 m
    cableVel: 0, // m/s
    drumPos: 0, // 卷筒角 rad
    drumVel: 0, // rad/s
    lastTension: 0,
  };

  /** 驱动器一个控制周期（默认 1 kHz）：位置/速度/电流三环，返回电磁力矩 */
  function stepDriver(dt) {
    st.cmdPosCable = st.steps * cablePerStep;
    const cmdMotor = (st.cmdPosCable / r) * G;
    st.followingErr = cmdMotor - st.motorPos;
    if (!st.enabled || st.fault) {
      st.curRef = 0;
      st.cur *= Math.exp(-dt / 5e-4);
      st.motorTrq = 0;
      st.velInt = 0;
      return 0;
    }
    let vCmd = 0;
    if (st.mode === MODE_POS) {
      // 位置环（P）输出速度指令，限幅在驱动器最大速度
      vCmd = clamp(st.followingErr * a.position_kp, -a.velocity_max, a.velocity_max);
    } else if (st.mode === MODE_VEL || st.mode === MODE_TRQ_VEL) {
      vCmd = (st.cmdVelCable / r) * G;
    }
    const verr = vCmd - st.motorVel;
    st.velInt = clamp(st.velInt + verr * dt * a.velocity_ki, -a.current_limit, a.current_limit);
    let cur = a.velocity_kp * verr + st.velInt;
    if (st.mode === MODE_TRQ || st.mode === MODE_TRQ_VEL) {
      cur += st.cmdTrqMotor / a.torque_constant;
    }
    // 反电动势限压（母线 48 V）
    const be = Math.abs(st.motorVel) * a.back_emf;
    const curMax = Math.min(a.current_limit, Math.max(0, (a.current_limit * (a.bus_voltage - be)) / a.bus_voltage));
    st.curRef = clamp(cur, -curMax, curMax);
    // 电流环一阶
    const alpha = 1 - Math.exp(-2 * PI * a.current_bandwidth * dt);
    st.cur += (st.curRef - st.cur) * alpha;
    // 电磁力矩（高速时转矩下降）
    const spdDrop = 1 / Math.sqrt(1 + (st.motorVel / a.corner_vel) ** 2);
    st.motorTrq = clamp(st.cur * a.torque_constant * spdDrop, -tHold, tHold);
    // 过载统计：负载力矩超出保持能力（不直接报警，便于观察「拉不住」的现象）
    if (Math.abs(st.loadTrq) > tHold * 1.02) st.stallEvents++;
    return st.motorTrq;
  }

  /**
   * 物理子步：电机轴 + 卷筒转动动力学（负载由平台动力学给出）。
   * @param {number} h s
   * @param {number} loadTrqMotor 绳张力折算到电机轴的阻力矩 Nm（收绳方向为正）
   * @param {number} Jload 负载折算到电机轴的惯量 kg·m²（张力刚度贡献）
   */
  function stepDynamics(h, loadTrqMotor, Jload) {
    st.loadTrq = loadTrqMotor;
    const J = a.inertia_motor + a.inertia_gear + Math.max(0, Jload);
    st.drumJLoad = Math.max(0, Jload);
    let trq = st.motorTrq;
    // 摩擦（保持力矩的 2%）：低速用线性化，模拟粘滑过渡
    const qs = a.qstop_torque;
    const wEnv = 1e-2;
    let fric = 0;
    if (st.enabled && !st.fault) fric = Math.abs(st.motorVel) > wEnv ? qs * Math.sign(st.motorVel) : (qs * st.motorVel) / wEnv;
    const acc = (trq - loadTrqMotor - fric) / Math.max(J, 1e-9);
    st.motorAcc = acc;
    st.motorVel += acc * h;
    st.motorPos += st.motorVel * h;
    st.drumPos = st.motorPos / G;
    st.drumVel = st.motorVel / G;
    st.cablePos = st.drumPos * r;
    st.cableVel = st.drumVel * r;
  }

  /**
   * 控制器下发：cmd = { mode, pos(绝对绳长 m), vel(m/s), trq(Nm 电机轴) }
   * l0 为初始绳长，用于把「绝对绳长」换算成「卷筒收绳量」再换算成脉冲数。
   */
  function command(cmd, l0) {
    if (!cmd) return;
    st.mode = cmd.mode ?? MODE_POS;
    const takeup = (l0 ?? 0) - cmd.pos;
    st.steps = Math.round(takeup / cablePerStep);
    st.cmdVelCable = cmd.vel ?? 0;
    st.cmdTrqMotor = cmd.trq ?? 0;
  }

  return {
    st, stepDriver, stepDynamics, command,
    spr, motorRadPerStep, cablePerStep, cablePerMotorRad: r / G, tHoldOut: tHold * G * eff,
  };
}

function clamp(v, lo, hi) {
  return v < lo ? lo : v > hi ? hi : v;
}
