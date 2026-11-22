@echo off
setlocal
pushd "%~dp0..\.." || exit /b 1
if not exist "build\host\CMakeCache.txt" (
    cmake -S host -B build\host -G Ninja -DCMAKE_C_COMPILER=gcc
    if errorlevel 1 goto failed
)
cmake --build build\host --target baize_panel_io --config Release
if errorlevel 1 goto failed
where pythonw.exe >nul 2>nul
if errorlevel 1 (
    echo Python 3.10+ with Tk and pythonw.exe must be on PATH.
    goto failed
)
start "" pythonw.exe -B "%~dp0baize-panel.pyw" %*
popd
exit /b 0

:failed
popd
pause
exit /b 1
