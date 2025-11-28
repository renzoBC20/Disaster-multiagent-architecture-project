@echo off
REM Script para probar el controlador UAV inteligente
REM Ejecutar en una terminal PowerShell después de activar el entorno ROS 2

echo ========================================
echo   Probando Controlador UAV Inteligente
echo ========================================
echo.

REM Verificar que estamos en el directorio correcto
if not exist "scripts\uav_ai_controller.py" (
    echo ERROR: No se encuentra uav_ai_controller.py
    echo Asegurate de estar en el directorio microsim
    pause
    exit /b 1
)

REM Verificar variables de entorno ROS 2
echo Verificando entorno ROS 2...
ros2 --help >nul 2>&1
if errorlevel 1 (
    echo ERROR: ros2 no encontrado. Asegurate de:
    echo   1. Activar conda: conda activate ros2_humble
    echo   2. Cargar workspace: call install\setup.bat
    pause
    exit /b 1
)

echo ✅ ROS 2 detectado
echo.

REM Verificar que Python pueda importar los módulos necesarios
echo Verificando dependencias de Python...
python -c "import rclpy; import numpy; import cv2; print('✅ Dependencias básicas OK')" 2>nul
if errorlevel 1 (
    echo ERROR: Faltan dependencias. Instala:
    echo   conda install -c conda-forge opencv-python numpy
    pause
    exit /b 1
)

python -c "import langchain_openai; print('✅ LangChain OK')" 2>nul
if errorlevel 1 (
    echo ⚠️  ADVERTENCIA: LangChain no encontrado
    echo   Para usar análisis IA, instala:
    echo   pip install langchain-openai langgraph
    echo   Y configura OPENAI_API_KEY en .env
    echo.
)

echo.
echo ========================================
echo   Iniciando Controlador UAV Inteligente
echo ========================================
echo.
echo Instrucciones:
echo   1. Abre otra terminal y ejecuta: ros2 run microsim microsim_node
echo   2. Opcional: Abre otra terminal y ejecuta: python scripts\viz_2d.py
echo   3. Este script ejecutará el controlador inteligente
echo.
pause

REM Ejecutar el controlador
python scripts\uav_ai_controller.py

