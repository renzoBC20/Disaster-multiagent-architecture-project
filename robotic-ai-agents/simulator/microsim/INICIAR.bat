@echo off
REM Script simple en CMD para iniciar el simulador
REM Este script funciona mejor que PowerShell en Windows

echo ========================================
echo   Iniciando Simulador MicroSim
echo ========================================
echo.

REM Activar entorno conda
call "%LOCALAPPDATA%\miniforge3\condabin\conda.bat" activate ros2_humble
if errorlevel 1 (
    echo ERROR: No se pudo activar conda environment
    echo Asegurate de que miniforge esta instalado
    pause
    exit /b 1
)

REM Ir al directorio
cd /d "%~dp0"

REM Cargar workspace
call install\setup.bat
if errorlevel 1 (
    echo ERROR: No se pudo cargar workspace
    echo Verifica que hayas compilado con: colcon build --packages-select microsim
    pause
    exit /b 1
)

echo.
echo ========================================
echo   Iniciando simulador...
echo ========================================
echo.

REM Ejecutar simulador
ros2 run microsim microsim_node

pause

