@echo off
REM Script para verificar que la configuración esté correcta

echo ========================================
echo   Verificando Configuración
echo ========================================
echo.

REM Activar entorno conda
call "%LOCALAPPDATA%\miniforge3\condabin\conda.bat" activate ros2_humble

echo.
echo [1/5] Verificando archivo .env...
if exist "..\..\MultiAgent\.env" (
    echo ✅ Archivo .env encontrado
) else (
    echo ❌ Archivo .env NO encontrado en MultiAgent
    echo.
    echo Ejecuta: CREAR_ENV.bat
    pause
    exit /b 1
)

echo.
echo [2/5] Verificando dependencias de IA...
python -c "import langgraph; print('✅ langgraph instalado')" 2>nul
if errorlevel 1 (
    echo ❌ langgraph NO instalado
    echo Ejecuta: INSTALAR_DEPENDENCIAS.bat
    pause
    exit /b 1
)

python -c "import langchain_openai; print('✅ langchain_openai instalado')" 2>nul
if errorlevel 1 (
    echo ❌ langchain_openai NO instalado
    echo Ejecuta: INSTALAR_DEPENDENCIAS.bat
    pause
    exit /b 1
)

echo.
echo [3/5] Verificando carga del .env...
python -c "from dotenv import load_dotenv; import os; load_dotenv('../../MultiAgent/.env'); api_key = os.getenv('OPENAI_API_KEY'); print('✅ API Key cargada:', api_key[:15] + '...' if api_key and api_key != 'tu_api_key_aqui' else '❌ API Key NO configurada o es placeholder')"

echo.
echo [4/5] Verificando OpenCV...
python -c "import cv2; print('✅ opencv instalado')" 2>nul
if errorlevel 1 (
    echo ❌ opencv NO instalado
    echo Ejecuta: conda install -c conda-forge opencv -y
)

echo.
echo [5/5] Verificando ROS 2...
python -c "import rclpy; print('✅ rclpy instalado')" 2>nul
if errorlevel 1 (
    echo ❌ rclpy NO instalado
    echo Revisa tu instalación de ROS 2
)

echo.
echo ========================================
echo   Resumen de Verificación
echo ========================================
echo.
echo ✅ Si todas las verificaciones pasan, estás listo!
echo.
pause

