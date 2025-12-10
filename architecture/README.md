# Arquitectura Multi-Agente para Robots Reales

Esta carpeta contiene **únicamente los archivos necesarios para implementar la arquitectura multi-agente en robots físicos**, excluyendo todos los componentes relacionados con la simulación.

## 📁 Estructura

```
architecture/
├── agents/                          # Agentes LangGraph puros
│   ├── UAV_agent.py                 # Agente aéreo - workflow completo
│   ├── UGV_Agent.py                 # Agente terrestre - workflow completo
│   └── __init__.py
│
├── ros2_integration/                # Integración con ROS 2
│   ├── langgraph_integration.py     # Funciones de análisis de imágenes con GPT
│   ├── langgraph_workflow.py        # Workflows adaptados para ROS 2
│   ├── coordinate_transforms.py     # Transformación de coordenadas imagen ↔ mundo
│   ├── geometric_detection.py       # Detección geométrica (opcional)
│   ├── workflow_adapters.py         # Adaptadores de funciones para ROS 2
│   └── __init__.py
│
├── ros2_controllers/                # Controladores ROS 2 para robots reales
│   ├── uav_langgraph_controller.py  # Controlador UAV con LangGraph
│   ├── ugv_langgraph_controller.py  # Controlador UGV con LangGraph
│   ├── autonomous_drone_controller.py  # Controlador base para UAV
│   └── __init__.py
│
├── docs/                            # Documentación
│   └── MANUAL_IMPLEMENTACION.md     # Manual completo de implementación
│
├── requirements.txt                 # Dependencias Python
├── config.example                   # Archivo de configuración ejemplo
└── README.md                        # Este archivo
```

## 🚀 Componentes Principales

### 1. Agentes LangGraph (`agents/`)

Los workflows puros de LangGraph que implementan la lógica de decisión de los agentes:

- **`UAV_agent.py`**: Análisis de imágenes, detección de víctimas/obstáculos, planificación de rutas
- **`UGV_Agent.py`**: Recepción de misiones, detección de colisiones, ejecución de rescate

### 2. Integración ROS 2 (`ros2_integration/`)

Módulos que adaptan los workflows de LangGraph para funcionar con ROS 2:

- **`langgraph_integration.py`**: Funciones de análisis visual con GPT-4o, procesamiento de imágenes
- **`langgraph_workflow.py`**: Workflows StateGraph adaptados para tiempo real con ROS 2
- **`coordinate_transforms.py`**: Transformación entre coordenadas de píxeles (imagen) y coordenadas del mundo (metros)

### 3. Controladores ROS 2 (`ros2_controllers/`)

Nodos ROS 2 que ejecutan los workflows de LangGraph y controlan los robots:

- **`uav_langgraph_controller.py`**: Nodo ROS 2 para UAV que suscribe a cámaras/sensores y publica comandos de movimiento
- **`ugv_langgraph_controller.py`**: Nodo ROS 2 para UGV que recibe misiones y ejecuta rescates

## 📋 Requisitos

### Software Base

- **ROS 2 Humble** (o superior)
- **Python 3.8+**
- **OpenAI API Key** (para GPT-4o/GPT-5)

### Dependencias Python

```bash
pip install -r requirements.txt
```

Las dependencias principales incluyen:
- `langgraph>=0.6.0`
- `langchain-openai>=0.3.0`
- `opencv-python>=4.8.0`
- `numpy>=1.24.0`
- `python-dotenv>=1.0.0`

### Dependencias ROS 2

```bash
# En un workspace ROS 2
rosdep install --from-paths . --ignore-src -r -y
```

## ⚙️ Configuración

1. **Copiar archivo de configuración**:
   ```bash
   cp config.example .env
   ```

2. **Configurar API Key de OpenAI**:
   ```bash
   # Editar .env
   OPENAI_API_KEY=tu_api_key_aqui
   ```

3. **Ajustar parámetros según tu robot** (ver `docs/MANUAL_IMPLEMENTACION.md`)

## 🔌 Integración con Robots Reales

### Topics ROS 2 Requeridos

**Para UAV:**
- Suscribe: `/uav/camera/image_raw` (tipo `sensor_msgs/Image`)
- Suscribe: `/uav/odom` (tipo `nav_msgs/Odometry`)
- Publica: `/uav/cmd_vel` (tipo `geometry_msgs/Twist`)
- Publica: `/uav/mission_brief` (tipo `std_msgs/String`)

**Para UGV:**
- Suscribe: `/uav/mission_brief` (tipo `std_msgs/String`)
- Suscribe: `/rover/odom` (tipo `nav_msgs/Odometry`)
- Publica: `/rover/cmd_vel` (tipo `geometry_msgs/Twist`)

### Ejecución

**En el UAV:**
```bash
ros2 run tu_paquete uav_langgraph_controller
```

**En el UGV:**
```bash
ros2 run tu_paquete ugv_langgraph_controller
```

## 📚 Documentación Completa

Para más detalles sobre:
- Especificaciones de hardware
- Configuración detallada
- Troubleshooting
- Testing y validación

Ver: **`docs/MANUAL_IMPLEMENTACION.md`**

## 🔄 Diferencias con la Versión de Simulación

Esta versión **excluye**:
- ❌ Simulador MicroSim
- ❌ Escenarios de simulación (`scenarios/`)
- ❌ Scripts de inicio de simulación (`.bat`)
- ❌ Visualización 2D/3D
- ❌ Tests de simulación

**Incluye solo**:
- ✅ Workflows de LangGraph
- ✅ Integración ROS 2
- ✅ Controladores para robots reales
- ✅ Documentación de implementación

## 📝 Notas de Desarrollo

- Los controladores están diseñados para robots reales y requieren odometría precisa
- Las transformaciones de coordenadas asumen una cámara con parámetros conocidos (ver `coordinate_transforms.py`)
- El sistema requiere conexión a internet para las llamadas a la API de OpenAI

## 🤝 Soporte

Para problemas o preguntas, consultar:
- Manual de implementación: `docs/MANUAL_IMPLEMENTACION.md`
- README principal del proyecto: `../README.md`

