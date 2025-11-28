# 🚁 Integración MultiAgent - MicroSim

## 📋 Resumen del Proyecto

Este proyecto integra el sistema MultiAgent UAV-UGV (basado en LangGraph + OpenAI GPT) con el simulador ROS 2 MicroSim para crear un sistema completo de rescate autónomo.

---

## ✅ Estado Actual del Proyecto

### Fase 1: Estructura Base ✅ COMPLETADA

**Archivos creados:**
1. `multiagent/__init__.py` - Módulo principal
2. `multiagent/coordinate_transforms.py` - Transformación imagen ↔ mundo
3. `multiagent/langgraph_integration.py` - Funciones de análisis IA adaptadas
4. `scripts/uav_ai_controller.py` - Controlador UAV inteligente con ROS 2

**Funcionalidades implementadas:**
- ✅ Conversión de coordenadas entre imagen y mundo
- ✅ Integración básica con funciones LangGraph
- ✅ Análisis de víctimas y obstáculos con GPT
- ✅ Planificación de rutas inteligente
- ✅ Publicación de misiones al UGV vía ROS 2

---

## 🚀 Cómo Usar

### 1. Activar Entorno ROS 2

```powershell
# Activar conda environment
& "$env:LOCALAPPDATA\miniforge3\condabin\conda.bat" activate ros2_humble

# Cargar workspace
cd "D:\Proyectos PFG\robotic-ai-agents\simulator\microsim"
& install\setup.bat
```

### 2. Ejecutar Simulador

**Terminal 1 - Simulador:**
```powershell
ros2 run microsim microsim_node
```

**Terminal 2 - Visualización:**
```powershell
python scripts\viz_2d.py
```

**Terminal 3 - Controlador UAV Inteligente:**
```powershell
python scripts\uav_ai_controller.py
```

---

## 📁 Estructura del Proyecto

```
microsim/
├── multiagent/                          # 🆕 Módulo de integración
│   ├── __init__.py
│   ├── coordinate_transforms.py        # Transformación coordenadas
│   └── langgraph_integration.py        # Funciones IA adaptadas
├── scripts/
│   ├── autonomous_drone_controller.py  # Controlador básico (referencia)
│   ├── uav_ai_controller.py           # 🆕 Controlador UAV inteligente
│   └── viz_2d.py                       # Visualización
├── PROJECT_PLAN.md                     # 🆕 Plan detallado del proyecto
└── README_INTEGRATION.md               # 🆕 Este archivo
```

---

## 🔧 Componentes Principales

### 1. `CoordinateTransformer`
- Convierte coordenadas de píxel (imagen) a coordenadas mundo (metros)
- Maneja transformaciones 2D y 3D
- Soporte para diferentes resoluciones de imagen

### 2. `langgraph_integration`
Funciones adaptadas del sistema MultiAgent original:
- `identify_victims_from_image()` - Identifica víctimas usando GPT
- `identify_obstacles_from_image()` - Identifica obstáculos usando GPT
- `plan_route()` - Planifica rutas óptimas
- `convert_image_to_cv2()` - Convierte imágenes ROS 2 → OpenCV

### 3. `UAVAIController`
Controlador inteligente que:
- Extiende `AutonomousDroneController`
- Analiza imágenes de la cámara con IA
- Planifica rutas automáticamente
- Publica misiones al UGV

---

## 🔄 Flujo de Funcionamiento

1. **Captura de Imagen**
   - MicroSim publica imágenes en `/drone/camera/image_raw`
   - `UAVAIController` recibe las imágenes

2. **Análisis con IA**
   - Cada 2 segundos, se analiza la imagen más reciente
   - GPT identifica víctimas y obstáculos
   - Se genera una ruta optimizada

3. **Conversión de Coordenadas**
   - Coordenadas de imagen → Coordenadas mundo
   - Waypoints actualizados automáticamente

4. **Control del UAV**
   - El controlador sigue los waypoints generados
   - Usa la lógica de control básica mejorada

5. **Comunicación con UGV**
   - Publica misión en `/uav/mission_brief`
   - UGV puede suscribirse y ejecutar rescate

---

## 📝 Próximos Pasos

### Pendiente:
- [ ] Definir mensajes ROS 2 personalizados (MissionBrief.msg)
- [ ] Crear controlador UGV inteligente
- [ ] Implementar detección de colisiones con sensores ROS 2
- [ ] Mejorar visualización para mostrar decisiones IA
- [ ] Testing y validación

---

## ⚠️ Requisitos

- ROS 2 Humble (conda environment `ros2_humble`)
- Python 3.11
- Dependencias del sistema MultiAgent:
  - `langchain-openai`
  - `langgraph`
  - `opencv-python`
  - `cv_bridge`
- API Key de OpenAI configurada en `.env`

---

## 🐛 Troubleshooting

### Error: "No module named 'multiagent'"
- Asegúrate de estar en el directorio correcto
- Verifica que el path esté configurado en `uav_ai_controller.py`

### Error: "llm is None"
- Verifica que tengas configurado `OPENAI_API_KEY` en `.env`
- Instala `langchain-openai` y `langgraph`

### Error: "Package 'microsim' not found"
- Ejecuta `& install\setup.bat` después de activar conda
- Verifica que hayas compilado con `colcon build`

---

## 📚 Referencias

- [AI Controller Guide](docs/AI_CONTROLLER_GUIDE.md)
- [World Configuration](docs/WORLD_CONFIGURATION.md)
- [Project Plan](PROJECT_PLAN.md)

---

**Estado**: 🟢 Fase 1 completada - Listo para pruebas iniciales

