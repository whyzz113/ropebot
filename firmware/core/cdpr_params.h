/**
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
#define CDPR_TOPOLOGY_CORNER 1

#define CDPR_DEFAULT_PARAMS { \
    /* frame */ 1.03f, 1.03f, 1.18f, \
    /* platform */ 0.15f, 0.15f, 0.016f, 4.0f, 9.81f, \
    /* winders */ 0.325f, 0.325f, 1.1f, 0.1f, 0.02f, 0.04f, \
    /* corner topo */ 0.485f, 0.1f, 0.03f, \
    /* cables */ 1000000000.0f, 0.0015f, 3.0f, 200.0f, \
    /* actuators */ 12.0f, 1.8f, 10.0f, 0.85f, 16, \
    /* controller */ 0.45f, 1.2f, 6.0f, 1.0f, 0.675f, 100.0f, 8.0e-5f, 0.05f, \
    /* home pose */ { 0.0f, 0.6f, 0.0f, 0.0f, 0.0f, 0.0f } \
}

static inline cdpr_params_t cdpr_default_params(void) {
    const cdpr_params_t p = CDPR_DEFAULT_PARAMS;
    return p;
}

#endif
