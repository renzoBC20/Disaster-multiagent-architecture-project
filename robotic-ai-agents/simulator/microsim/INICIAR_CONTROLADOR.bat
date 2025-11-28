@echo off
REM Script simple en CMD para iniciar el controlador UAV inteligente
REM Este script funciona mejor que PowerShell en Windows

echo ========================================
echo   Controlador UAV Inteligente
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

REM Cargar workspace
call install\setup.bat
if errorlevel 1 (
    echo ERROR: No se pudo cargar workspace
    pause
    exit /b 1
)

echo.
echo ========================================
echo   Iniciando controlador...
echo ========================================
echo.
echo Asegurate de que el simulador esta corriendo!
echo.
pause

REM Ejecutar controlador
python scripts\uav_ai_controller.py

pause

