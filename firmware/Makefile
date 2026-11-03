/**
 * 构建脚本（Windows / MSVC 与 MinGW 通用）
 *
 *   make            编译 board_sim.exe（板级仿真，可在无硬件时运行）
 *   make test       编译并运行一致性测试（与 JS 核心对比）
 *   make clean      清理
 *
 * 若使用 MinGW：mingw32-make
 * 若使用 MSVC：nmake /f Makefile.msvc
 */
CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra -std=c11
LDFLAGS ?= -lm
OUT ?= build

SRC_CORE = core/cdpr_core.c core/cdpr_traj.c
SRC_APP  = node/control_main.c board_sim/main.c

all: $(OUT)/board_sim.exe

$(OUT)/board_sim.exe: $(SRC_CORE) $(SRC_APP)
	@if not exist "$(OUT)" mkdir "$(OUT)"
	$(CC) $(CFLAGS) -o $@ $(SRC_CORE) $(SRC_APP) $(LDFLAGS)

test: $(OUT)/consistency_test.exe
	$(OUT)/consistency_test.exe

$(OUT)/consistency_test.exe: $(SRC_CORE) tests/consistency_test.c
	@if not exist "$(OUT)" mkdir "$(OUT)"
	$(CC) $(CFLAGS) -o $@ $(SRC_CORE) tests/consistency_test.c $(LDFLAGS)

clean:
	@if exist "$(OUT)" rmdir /s /q "$(OUT)"

.PHONY: all test clean
