@echo off
REM Script simple en CMD para iniciar la visualizacion
REM Este script funciona mejor que PowerShell en Windows

echo ========================================
echo   Visualizacion MicroSim
echo ========================================
echo.

REM Activar entorno conda
call "%LOCALAPPDATA%\miniforge3\condabin\conda.bat" activate ros2_humble
if errorlevel 1 (
    echo ERROR: No se pudo activar conda environment
    pause
    exit /b 1
)

REM Ir al directorio
cd /d "%~dp0"

echo.
echo ========================================
echo   Iniciando visualizacion...
echo ========================================
echo.

REM Ejecutar visualizacion
python scripts\viz_2d.py

pause

