@echo off
REM Script para ejecutar el sistema completo UAV-UGV con LangGraph
REM Este script explica los pasos necesarios

echo ========================================
echo   Sistema Completo UAV-UGV con LangGraph
echo ========================================
echo.
echo Este sistema necesita 4 terminales ejecutandose simultaneamente:
echo.
echo   TERMINAL 1: Simulador
echo   TERMINAL 2: Visualizacion (opcional)
echo   TERMINAL 3: Controlador UAV con LangGraph
echo   TERMINAL 4: Controlador UGV con LangGraph
echo.
echo ========================================
echo   INSTRUCCIONES
echo ========================================
echo.
echo PASO 1: Abre una terminal y ejecuta:
echo   INICIAR.bat
echo   (Espera a que el simulador inicie completamente)
echo.
echo PASO 2 (Opcional): Abre otra terminal y ejecuta:
echo   INICIAR_VISUALIZACION.bat
echo   (Para ver el mundo en 3D)
echo.
echo PASO 3: Abre otra terminal y ejecuta:
echo   INICIAR_LANGGRAPH.bat
echo   (Espera a que el UAV analice y publique la mision)
echo.
echo PASO 4: Abre otra terminal y ejecuta:
echo   INICIAR_UGV.bat
echo   (El UGV seguira la ruta planificada por el UAV)
echo.
echo ========================================
echo.
echo Presiona cualquier tecla para abrir la documentacion...
pause >nul

echo.
echo ========================================
echo   ABRIENDO ARCHIVOS .bat INDIVIDUALES
echo ========================================
echo.
echo Ahora puedes ejecutar cada archivo .bat en una terminal diferente:
echo.
echo   1. INICIAR.bat
echo   2. INICIAR_VISUALIZACION.bat (opcional)
echo   3. INICIAR_LANGGRAPH.bat
echo   4. INICIAR_UGV.bat
echo.
echo ========================================
echo.

pause

