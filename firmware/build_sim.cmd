@echo off
chcp 65001 >nul
rem ---------------------------------------------------------------------------
rem Build board_sim.exe with MSVC (no MinGW required)
rem Usage: build_sim.cmd
rem ---------------------------------------------------------------------------
setlocal

set VSDIR=C:\Program Files\Microsoft Visual Studio\2022\Community
set VCVARS=%VSDIR%\VC\Auxiliary\Build\vcvars64.bat
if not exist "%VCVARS%" (
  echo [ERROR] vcvars64.bat not found under VS 2022 Community.
  echo         Edit VSDIR in this script, or use MinGW make instead.
  exit /b 1
)

call "%VCVARS%" >nul
if not exist build mkdir build

cl /nologo /O2 /W3 /utf-8 /D_CRT_SECURE_NO_WARNINGS /Fe:build\board_sim.exe core\cdpr_core.c core\cdpr_traj.c node\control_main.c board_sim\main.c /link /SUBSYSTEM:CONSOLE
if errorlevel 1 (
  echo [ERROR] board_sim build failed
  exit /b 1
)

cl /nologo /O2 /W3 /utf-8 /D_CRT_SECURE_NO_WARNINGS /Fe:build\consistency_test.exe core\cdpr_core.c core\cdpr_traj.c tests\consistency_test.c /link /SUBSYSTEM:CONSOLE
if errorlevel 1 (
  echo [ERROR] consistency test build failed
  exit /b 1
)

echo [OK] build\board_sim.exe and build\consistency_test.exe
endlocal
