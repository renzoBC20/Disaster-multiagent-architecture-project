@echo off
REM Script para iniciar el controlador UAV con LangGraph completo

echo ========================================
echo   Controlador UAV con LangGraph
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
echo   Iniciando controlador con LangGraph...
echo ========================================
echo.
echo IMPORTANTE: Asegurate de que:
echo   1. El simulador este corriendo (Terminal 1)
echo   2. Este controlador usa el workflow completo de LangGraph
echo      con patrullaje y StateGraph
echo.

pause

REM Ejecutar controlador
python scripts\uav_langgraph_controller.py

pause

