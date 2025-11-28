@echo off
REM Script para crear el archivo .env con la estructura correcta
REM Este archivo está en .gitignore, por lo que es seguro

echo ========================================
echo   Creando archivo .env
echo ========================================
echo.

REM Definir la ruta del archivo .env
set "ENV_FILE=..\..\MultiAgent\.env"

REM Verificar si ya existe
if exist "%ENV_FILE%" (
    echo El archivo .env ya existe!
    echo.
    echo Quieres sobrescribirlo? (S/N)
    choice /C SN /N /M "> "
    if errorlevel 2 goto :end
    echo.
)

echo Creando archivo .env...
echo.

REM Crear el archivo .env con la estructura
(
    echo # Configuración del Sistema Multi-Agente UAV-UGV
    echo # Sistema de Integración MultiAgent con MicroSim
    echo.
    echo # OpenAI API Configuration
    echo OPENAI_API_KEY=tu_api_key_aqui
    echo OPENAI_MODEL=gpt-5-mini
    echo OPENAI_TEMPERATURE=0.1
    echo.
    echo # LangGraph Configuration ^(opcional^)
    echo LANGGRAPH_API_KEY=
    echo.
    echo # Video Processing Configuration
    echo VIDEO_PATH=uav_simulation.mp4
    echo FRAME_EXTRACTION_INTERVAL=30
    echo MAX_FRAMES_TO_ANALYZE=10
    echo.
    echo # UAV Agent Configuration
    echo UAV_ANALYSIS_CONFIDENCE=0.7
    echo UAV_OBSTACLE_DETECTION=True
    echo UAV_ROUTE_OPTIMIZATION=True
    echo.
    echo # UGV Agent Configuration
    echo UGV_COLLISION_DETECTION=True
    echo UGV_SAFETY_DISTANCE=2.0
    echo UGV_RECALCULATION_THRESHOLD=0.5
    echo.
    echo # Logging Configuration
    echo LOG_LEVEL=INFO
    echo LOG_FILE=logs/system.log
    echo ENABLE_CONSOLE_LOGGING=True
    echo.
    echo # Communication Configuration
    echo MESSAGE_FILE=uav_to_ugv_message.json
    echo COMMUNICATION_TIMEOUT=30
    echo.
    echo # Performance Configuration
    echo MAX_CONCURRENT_ANALYSES=3
    echo CACHE_ANALYSIS_RESULTS=True
    echo CACHE_DURATION=3600
) > "%ENV_FILE%"

echo ✅ Archivo creado en: %ENV_FILE%
echo.
echo IMPORTANTE: Edita el archivo y reemplaza 'tu_api_key_aqui' con tu API key real
echo.
echo Para obtener tu API key: https://platform.openai.com/api-keys
echo.

:end
pause

