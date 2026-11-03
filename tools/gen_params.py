#!/usr/bin/env python3
"""由 sim/core/cdpr_params.json 生成 firmware/core/cdpr_params.h。

用法：
    python tools/gen_params.py

改动 JSON 参数后运行本脚本，再重新编译固件，即可让 C 侧与 JS 侧完全一致。
"""
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
JSON_PATH = os.path.join(ROOT, 'sim', 'core', 'cdpr_params.json')
H_PATH = os.path.join(ROOT, 'firmware', 'core', 'cdpr_params.h')

TEMPLATE = '''/**
 * 由 sim/core/cdpr_params.json 自动生成，请勿手工编辑。
 * 重新生成：python tools/gen_params.py
 */
#ifndef CDPR_PARAMS_H
#define CDPR_PARAMS_H

#include "cdpr_core.h"

#ifdef CDPR_TOPOLOGY_CORNER
#undef CDPR_TOPOLOGY_CORNER
#endif
/* 拓扑：1 = 角点（8 台电机在框架 8 角 + 平台四边中点系绳）
 *       0 = 标准（锚点在四边中点 + 同边双角点系绳） */
#define CDPR_TOPOLOGY_CORNER {topo_corner}

#define CDPR_DEFAULT_PARAMS {{ \\
    /* frame */ {frame_w}f, {frame_d}f, {frame_h}f, \\
    /* platform */ {plat_hx}f, {plat_hz}f, {plat_t}f, {plat_m}f, {grav}f, \\
    /* winders */ {wind_x}f, {wind_z}f, {wind_up}f, {wind_lo}f, {drum_r}f, {edge_off}f, \\
    /* corner topo */ {corner_sp}f, {corner_off}f, {tether_off}f, \\
    /* cables */ {emod}f, {diam}f, {tmin}f, {tmax}f, \\
    /* actuators */ {hold}f, {step_deg}f, {gear}f, {gear_eff}f, {micro}, \\
    /* controller */ {vmax}f, {amax}f, {amax_ang}f, 1.0f, {lref_vmax}f, {lref_amax}f, {tension_kp}f, {floor_y}f, \\
    /* home pose */ {{ {h0}f, {h1}f, {h2}f, {h3}f, {h4}f, {h5}f }} \\
}}

static inline cdpr_params_t cdpr_default_params(void) {{
    const cdpr_params_t p = CDPR_DEFAULT_PARAMS;
    return p;
}}

#endif
'''


def lit(v):
    """C 浮点字面量：保证有小数点，避免 `4f` 这种非法后缀。"""
    if isinstance(v, int):
        return f'{v}.0'
    s = repr(float(v))
    if 'e' in s or 'E' in s:
        mant, exp = s.lower().split('e')
        if '.' not in mant:
            mant += '.0'
        return f'{mant}e{int(exp)}'
    if '.' not in s:
        s += '.0'
    return s


def main():
    with open(JSON_PATH, encoding='utf-8') as f:
        p = json.load(f)

    fw = p['frames']
    pl = p['platform']
    wd = p['winders']
    cb = p['cables']
    ac = p['actuators']
    ct = p['controller']
    home = ct['home_pose']

    text = TEMPLATE.format(
        topo_corner=1 if wd.get('topology') == 'corner' else 0,
        frame_w=lit(fw['width']), frame_d=lit(fw['depth']), frame_h=lit(fw['top_y']),
        plat_hx=lit(pl['half_size_x']), plat_hz=lit(pl['half_size_z']),
        plat_t=lit(pl['thickness']), plat_m=lit(pl['mass']), grav=lit(pl['gravity']),
        wind_x=lit(wd['half_spacing_x']), wind_z=lit(wd['half_spacing_z']),
        wind_up=lit(wd['upper_y']), wind_lo=lit(wd['lower_y']), drum_r=lit(wd['drum_radius']),
        edge_off=lit(wd.get('edge_offset', 0.04)),
        corner_sp=lit(wd.get('corner_spacing', 0.485)),
        corner_off=lit(wd.get('corner_offset', 0.10)),
        tether_off=lit(wd.get('tether_offset', 0.03)),
        emod=lit(cb['emodulus']), diam=lit(cb['diameter']), tmin=lit(cb['t_min']), tmax=lit(cb['t_max']),
        hold=lit(ac['hold_torque']), step_deg=lit(ac['step_angle_deg']),
        gear=lit(ac['gear_ratio']), gear_eff=lit(ac['gear_efficiency']), micro=ac['driver_microstep'],
        vmax=lit(ct['max_trans_vel']), amax=lit(ct['max_trans_acc']), amax_ang=lit(ct['max_ang_acc']),
        lref_vmax=lit(ct.get('lref_vmax', ct['max_trans_vel'] * 1.5)),
        lref_amax=lit(ct.get('lref_amax', 100.0)),
        tension_kp=lit(ct['tension_kp']), floor_y=lit(ct['floor_y']),
        h0=lit(home[0]), h1=lit(home[1]), h2=lit(home[2]),
        h3=lit(home[3]), h4=lit(home[4]), h5=lit(home[5]),
    )

    with open(H_PATH, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text)
    topo = '角点（8 电机在框架 8 角）' if wd.get('topology') == 'corner' else '标准（四边中点）'
    print(f'已生成 {H_PATH}')
    print(f'  拓扑: {topo}')
    print(f'  卷筒 R={wd["drum_radius"]*1000:.0f} mm, 减速比 {ac["gear_ratio"]}, 细分 {ac["driver_microstep"]}')
    print(f'  上/下卷筒高度 {wd["upper_y"]:.2f} / {wd["lower_y"]:.2f} m, 零位 y={home[1]:.2f} m')
    print('  下一步：重新编译固件（firmware/build_sim.cmd）')


if __name__ == '__main__':
    sys.exit(main())
