# 📦 Instalación de Dependencias

## ⚠️ Problema Detectado

El error que viste indica que faltan dependencias:

```
⚠️ Advertencia: LangGraph no disponible: No module named 'langgraph'
⚠️ Advertencia: MultiAgent modules no disponibles: No module named 'UAV_agent'
```

---

## 🔧 Solución: Instalar Dependencias

### Método 1: Script Automático (Recomendado)

Ejecuta este script en CMD:

```cmd
cd "D:\Proyectos PFG\robotic-ai-agents\simulator\microsim"
INSTALAR_DEPENDENCIAS.bat
```

Esto instalará automáticamente:
- `langgraph`
- `langchain-openai`
- `langchain-core`
- `python-dotenv`
- `pydantic`

---

### Método 2: Manual

Si prefieres hacerlo manual:

```cmd
REM Activar entorno
call "%LOCALAPPDATA%\miniforge3\condabin\conda.bat" activate ros2_humble

REM Instalar dependencias
pip install langgraph langchain-openai langchain-core python-dotenv pydantic

REM Verificar
python -c "import langgraph; print('✅ OK')"
```

---

## ✅ Verificación

Después de instalar, verifica que todo esté bien:

```cmd
call "%LOCALAPPDATA%\miniforge3\condabin\conda.bat" activate ros2_humble

python -c "import langgraph; import langchain_openai; import cv2; print('✅ Todo OK')"
```

---

## 🎯 Importante

Estas dependencias deben instalarse en el entorno `ros2_humble`, no en el sistema global.

Si instalaste en otro lugar, desinstálalas:
```cmd
pip uninstall langgraph langchain-openai -y
```

Y reinstálalas en el entorno correcto.

---

**¿Listo?** Ejecuta `INSTALAR_DEPENDENCIAS.bat` ahora 🚀

