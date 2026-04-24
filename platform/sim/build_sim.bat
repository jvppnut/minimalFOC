@echo off
setlocal

set ROOT=%~dp0..\..
set OUT=%~dp0foc_sim.dll

gcc -shared -O2 ^
    -I"%ROOT%" ^
    "%ROOT%\core\math\foc_math.c" ^
    "%ROOT%\core\math\foc_pid.c" ^
    "%ROOT%\foc.c" ^
    "%ROOT%\platform\sim\foc_sim_interface.c" ^
    -o "%OUT%" ^
    -lm

if %ERRORLEVEL% neq 0 (
    echo Build FAILED.
    exit /b %ERRORLEVEL%
)

echo Built %OUT%
