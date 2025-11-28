@echo off
REM Script para compilar el paquete microsim

echo ========================================
echo   Compilando paquete microsim
echo ========================================
echo.

REM Activar entorno conda
call "%LOCALAPPDATA%\miniforge3\condabin\conda.bat" activate ros2_humble

REM Cargar workspace
call install\setup.bat

REM Compilar
colcon build --packages-select microsim --merge-install

if errorlevel 1 (
    echo.
    echo ERROR: La compilacion fallo
    pause
    exit /b 1
)

echo.
echo ========================================
echo   Compilacion exitosa!
echo ========================================
echo.

pause

