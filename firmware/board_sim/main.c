/**
 * board_sim：STM32 控制代码的「无板仿真」宿主。
 *
 * 同一份控制代码（firmware/core + firmware/node）在 PC 上编译运行，
 * 通过 stdio（按行 JSON）与仿真服务器通信：
 *   上位机 -> 本程序：{"cmd":"step","n":1}  {"cmd":"goto","q":[...]}
 *   本程序 -> 上位机：{"type":"log","text":"..."}  {"type":"cmd","i":0,"len":0.647,"trq":-0.02}
 *
 * 这样可以在没有硬件的情况下，验证控制律、状态机、串口协议与参数表是否自洽。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "../core/cdpr_core.h"
#include "../core/cdpr_traj.h"
#include "../core/cdpr_params.h"
#include "../node/control_main.h"

int main(int argc, char **argv) {
    cdpr_params_t params = cdpr_default_params();
    cdpr_ctl_t ctl;
    cdpr_traj_t traj;
    ctl_boot_t boot;
    ctl_init(&boot, &ctl, &traj, &params);

    fprintf(stdout, "{\"type\":\"ready\",\"cables\":8,\"rate\":%d}\n", CTL_RATE_HZ);
    fflush(stdout);

    char line[4096];
    while (fgets(line, sizeof(line), stdin)) {
        /* 极简 JSON 解析：只认我们自己的命令，避免引入依赖 */
        if (strstr(line, "\"cmd\":\"step\"")) {
            int n = 1;
            const char *p = strstr(line, "\"n\":");
            if (p) n = atoi(p + 4);
            if (n < 1) n = 1;
            if (n > 20000) n = 20000;
            for (int i = 0; i < n; i++) {
                ctl_tick(&boot, &ctl, &traj);
            }
            /* 回传 8 根绳的指令 */
            fprintf(stdout, "{\"type\":\"cmd\",\"t\":%.4f,\"q\":[", (double)boot.sim_time);
            for (int k = 0; k < 6; k++) fprintf(stdout, "%s%.5f", k ? "," : "", (double)ctl.q_target[k]);
            fprintf(stdout, "],\"len\":[");
            for (int i = 0; i < 8; i++) fprintf(stdout, "%s%.6f", i ? "," : "", (double)ctl.len_ref[i]);
            fprintf(stdout, "],\"trq\":[");
            for (int i = 0; i < 8; i++) fprintf(stdout, "%s%.5f", i ? "," : "", (double)ctl.tension_ref[i]);
            fprintf(stdout, "],\"feasible\":%d}\n", ctl.feasible ? 1 : 0);
            fflush(stdout);
        } else if (strstr(line, "\"cmd\":\"goto\"")) {
            float q[6];
            const char *p = strstr(line, "\"q\":[");
            if (p) {
                sscanf(p + 5, "%f,%f,%f,%f,%f,%f", &q[0], &q[1], &q[2], &q[3], &q[4], &q[5]);
                traj_goto(&traj, q, 2.5f);
                fprintf(stdout, "{\"type\":\"log\",\"text\":\"goto ok\"}\n");
            }
        } else if (strstr(line, "\"cmd\":\"circle\"")) {
            traj_circle(&traj);
            fprintf(stdout, "{\"type\":\"log\",\"text\":\"circle ok\"}\n");
        } else if (strstr(line, "\"cmd\":\"pallet\"")) {
            traj_pallet(&traj, &params);
            fprintf(stdout, "{\"type\":\"log\",\"text\":\"pallet ok\"}\n");
        } else if (strstr(line, "\"cmd\":\"hold\"")) {
            traj_hold(&traj);
            fprintf(stdout, "{\"type\":\"log\",\"text\":\"hold ok\"}\n");
        } else if (strstr(line, "\"cmd\":\"quit\"")) {
            break;
        }
        fflush(stdout);
    }
    return 0;
}
