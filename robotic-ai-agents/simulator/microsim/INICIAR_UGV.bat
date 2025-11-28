@echo off
REM Script para iniciar el controlador UGV inteligente

echo ========================================
echo   Controlador UGV Inteligente
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
echo   Iniciando controlador UGV...
echo ========================================
echo.
echo IMPORTANTE: Asegurate de que:
echo   1. El simulador este corriendo (Terminal 1)
echo   2. El controlador UAV haya publicado una misión
echo.

pause

REM Ejecutar controlador UGV con LangGraph
python scripts\ugv_langgraph_controller.py

pause

