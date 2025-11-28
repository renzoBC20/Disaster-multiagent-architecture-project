@echo off
REM Script para instalar todas las dependencias necesarias

echo ========================================
echo   Instalando Dependencias
echo ========================================
echo.

REM Activar entorno conda
call "%LOCALAPPDATA%\miniforge3\condabin\conda.bat" activate ros2_humble
if errorlevel 1 (
    echo ERROR: No se pudo activar conda environment
    pause
    exit /b 1
)

echo.
echo Instalando paquetes de IA...
echo.

pip install langgraph langchain-openai langchain-core python-dotenv pydantic
if errorlevel 1 (
    echo ERROR al instalar dependencias de IA
    pause
)

echo.
echo Instalando cv_bridge (opcional)...
conda install -c conda-forge cv_bridge -y

echo.
echo ========================================
echo   Verificando instalación...
echo ========================================
echo.

python -c "import langgraph; print('✅ langgraph OK')" 2>nul
if errorlevel 1 echo ❌ langgraph NO instalado

python -c "import langchain_openai; print('✅ langchain_openai OK')" 2>nul
if errorlevel 1 echo ❌ langchain_openai NO instalado

python -c "import cv2; print('✅ opencv OK')" 2>nul
if errorlevel 1 echo ❌ opencv NO instalado

python -c "import dotenv; print('✅ python-dotenv OK')" 2>nul
if errorlevel 1 echo ❌ python-dotenv NO instalado

python -c "import rclpy; print('✅ rclpy OK')" 2>nul
if errorlevel 1 echo ❌ rclpy NO instalado

echo.
echo ========================================
echo   Instalación completada!
echo ========================================
echo.

pause
