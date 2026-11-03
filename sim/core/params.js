/**
 * 参数读取与合并（浏览器 / Node 通用）。
 */
import { mergeParams, validateParams } from './core.js';

export const PARAMS_PATH = new URL('./cdpr_params.json', import.meta.url);

export async function loadParamsJson() {
  const isNode = typeof process !== 'undefined' && process.versions && process.versions.node;
  if (isNode) {
    const fs = await import('node:fs');
    return JSON.parse(fs.readFileSync(PARAMS_PATH, 'utf8'));
  }
  const res = await fetch(PARAMS_PATH, { cache: 'no-store' });
  if (!res.ok) throw new Error(`读取 ${PARAMS_PATH} 失败：HTTP ${res.status}`);
  return await res.json();
}

/** 代码内置兜底参数：JSON 缺失或损坏时仍能启动 */
export function fallbackParams() {
  return {
    frames: { width: 0.8, depth: 0.8, height: 1.2, top_y: 1.08 },
    platform: { half_size_x: 0.075, half_size_z: 0.075, thickness: 0.016, mass: 4, gravity: 9.81 },
    winders: { half_spacing_x: 0.325, half_spacing_z: 0.325, upper_y: 1.0, lower_y: 0.725, drum_radius: 0.02 },
    cables: { k_preload: 50000, t_min: 5, t_max: 200, emodulus: 1e9, diameter: 0.0015 },
    actuators: {
      hold_torque: 12, step_angle_deg: 1.8, driver_microstep: 16, limit_current_ratio: 1,
      encoder_counts_per_rev: 10000, gear_ratio: 10, gear_efficiency: 0.85,
      inertia_motor: 0.0006, inertia_gear: 0.0004, qstop_torque: 0.25, velocity_kp: 0.25,
      velocity_ki: 60, position_kp: 40, velocity_max: 200, corner_vel: 30, current_limit: 5,
      torque_constant: 1.65, current_bandwidth: 600, back_emf: 0.6, bus_voltage: 48,
    },
    loop: { controller_rate_hz: 1000, driver_rate_hz: 1000, physics_rate_hz: 2000, waypoint_rate_hz: 100, log_rate_hz: 1000 },
    controller: {
      tension_alloc: 1, allocation_iterations: 30, seed_index: 0, tension_share: 0.15, tension_kp: 4e-5,
      waypoint_mode: 0, home_pose: [0, 0.25, 0, 0, 0, 0], max_trans_vel: 0.45, max_ang_vel: 0.8,
      max_trans_acc: 1.2, max_ang_acc: 3, workspace_margins: [0.1, 0.03, 0.1, 0.1, 0.2, 0.15],
      limits: { roll_rad: 0.45, pitch_rad: 0.45, yaw_rad: 3.1 },
      wrench_kp: 120, wrench_kd: 12, stiffness_log: true,
      pid: {
        enable: false, d_filter_hz: 8, force_limit_ratio: 0.8, moment_limit_ratio: 0.8,
        trans: { kp: 400, ki: 40, kd: 30 },
        rot: { kp: 6, ki: 1.2, kd: 0.8 },
      },
    },
    trajectory: {
      type: 'circle', radius: 0.12, height: 0.3, period: 10, xy_amplitude: 0.12, z_amplitude: 0.05,
      pitch_amplitude: 0.15, roll_amplitude: 0, yaw_amplitude: 0.5, segment_time: 3,
    },
    palletizing: { spacing: 0.15, count: 4, pick_x0: -0.18, pick_height: 0.28, transfer_height: 0.38, segment_time: 1.5 },
    poses: [{ name: 'home', p: [0, 0.3, 0], rpy: [0, 0, 0] }],
  };
}

/**
 * @param {object} overrides 局部覆盖（例如 { actuators: { gear_ratio: 20 } }）
 * @returns {Promise<{params:object, errors:string[], loaded:boolean}>}
 */
export async function readParams(overrides) {
  let base = null;
  let loaded = false;
  try {
    base = await loadParamsJson();
    loaded = true;
  } catch (e) {
    base = null;
  }
  const params = mergeParams(mergeParams(fallbackParams(), base), overrides || {});
  return { params, errors: validateParams(params), loaded };
}
