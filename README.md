# Disaster Multi-Agent Architecture Project

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![LangGraph](https://img.shields.io/badge/LangGraph-0.6.0+-green.svg)](https://python.langchain.com/docs/langgraph)

Sistema multi-agente completo para operaciones de rescate en desastres, integrando un UAV (vehículo aéreo no tripulado) y un UGV (vehículo terrestre no tripulado) con capacidades de inteligencia artificial avanzadas.

## 🎯 Descripción del Proyecto

Este proyecto implementa una arquitectura multi-agente completa que combina:

- **Sistema de Simulación ROS 2 (MicroSim)**: Simulador determinístico en tiempo real para drones y rovers
- **Agentes Inteligentes con LangGraph**: Sistema de planificación y ejecución basado en LLMs
- **Análisis Visual con GPT-4o/GPT-5**: Identificación automática de víctimas y obstáculos mediante visión artificial
- **Comunicación Inter-Agente**: Coordinación entre UAV y UGV mediante mensajes estructurados
- **Arquitectura para Robots Reales**: Implementación lista para desplegar en robots físicos

### 🎯 Objetivo Principal

Desarrollar y validar un sistema multi-agente autónomo que pueda:
1. **Simular** el comportamiento completo en un entorno virtual
2. **Desplegar** la misma arquitectura en robots reales sin modificaciones mayores

Por esta razón, el proyecto está organizado en:
- `architecture/`: Código limpio para robots reales (sin simulación)
- `robotic-ai-agents/`: Simulador completo para desarrollo y validación
- `MultiAgent/`: Agentes LangGraph originales para desarrollo

## 🏗️ Arquitectura del Sistema

```
┌─────────────────────────────────────────────────────────────┐
│                    MicroSim Simulator                        │
│  (ROS 2 - Deterministic Real-time Simulation)               │
│  - Drone (6-DOF kinematic)                                   │
│  - Rover (Differential-drive)                               │
│  - Sensors: GPS, Camera, Range                              │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼
┌─────────────────────────────────────────────────────────────┐
│              UAV Agent (LangGraph Workflow)                 │
│  - Video Analysis                                            │
│  - Victim & Obstacle Detection (GPT-4o Vision)              │
│  - Route Planning                                            │
│  - Mission Briefing Generation                               │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼ (JSON Messages)
┌─────────────────────────────────────────────────────────────┐
│              UGV Agent (LangGraph Workflow)                 │
│  - Mission Reception                                         │
│  - Collision Detection                                       │
│  - Path Replanning                                           │
│  - Victim Rescue Execution                                   │
└─────────────────────────────────────────────────────────────┘
```

## 📁 Estructura del Proyecto

El proyecto está organizado en tres componentes principales:

```
.
├── architecture/                         # 🎯 ARQUITECTURA PARA ROBOTS REALES
│   ├── agents/                          # Agentes LangGraph puros
│   │   ├── UAV_agent.py                 # Workflow completo del UAV
│   │   └── UGV_Agent.py                 # Workflow completo del UGV
│   ├── ros2_integration/                # Integración con ROS 2
│   │   ├── langgraph_integration.py     # Análisis de imágenes con GPT
│   │   ├── langgraph_workflow.py        # Workflows adaptados para ROS 2
│   │   ├── coordinate_transforms.py     # Transformación de coordenadas
│   │   ├── geometric_detection.py       # Detección geométrica (opcional)
│   │   └── workflow_adapters.py         # Adaptadores para ROS 2
│   ├── ros2_controllers/                # Controladores ROS 2
│   │   ├── uav_langgraph_controller.py  # Controlador UAV para robots reales
│   │   ├── ugv_langgraph_controller.py  # Controlador UGV para robots reales
│   │   └── autonomous_drone_controller.py # Controlador base UAV
│   ├── docs/                            # Documentación de implementación
│   │   └── MANUAL_IMPLEMENTACION.md     # Manual completo para robots reales
│   ├── requirements.txt                 # Dependencias Python
│   ├── config.example                   # Archivo de configuración
│   └── README.md                        # Guía de la arquitectura
│
├── MultiAgent/                          # 🔧 DESARROLLO Y TESTING
│   ├── UAV_agent.py                     # Agente aéreo (desarrollo original)
│   ├── UGV_Agent.py                     # Agente terrestre (desarrollo original)
│   ├── video_sim.py                     # Simulador de video para testing
│   ├── requirements.txt                 # Dependencias Python
│   ├── docs/                            # Documentación de desarrollo
│   └── venv/                            # Entorno virtual Python
│
├── robotic-ai-agents/                   # 🎮 SIMULADOR ROS 2 (MicroSim)
│   └── simulator/
│       └── microsim/                     # Paquete MicroSim completo
│           ├── microsim/                 # Nodos ROS 2 del simulador
│           │   ├── microsim_node.py      # Nodo principal del simulador
│           │   ├── camera.py             # Renderizado de cámara
│           │   ├── world.py              # Modelo del mundo
│           │   └── ...                   # Otros módulos del simulador
│           ├── scripts/                  # Controladores para simulación
│           │   ├── uav_langgraph_controller.py # Controlador UAV (simulación)
│           │   ├── ugv_langgraph_controller.py # Controlador UGV (simulación)
│           │   └── autonomous_drone_controller.py # Controlador base
│           ├── multiagent/               # Integración LangGraph con simulación
│           │   ├── langgraph_integration.py # Funciones de análisis
│           │   ├── coordinate_transforms.py # Transformaciones
│           │   └── ...                   # Otros módulos de integración
│           ├── scenarios/                # Escenarios de simulación
│           │   └── default.yaml          # Escenario por defecto
│           ├── docs/                     # Documentación técnica
│           ├── test/                     # Tests unitarios
│           └── *.bat                     # Scripts de inicio (Windows)
│
├── docs/                                # 📚 DOCUMENTACIÓN GENERAL
│   └── MANUAL_IMPLEMENTACION.md         # Manual de implementación (copia)
│
└── README.md                            # Este archivo
```

### 📦 Descripción de Carpetas Principales

#### 🎯 `architecture/` - Arquitectura para Robots Reales
**Propósito**: Contiene únicamente los archivos necesarios para implementar la arquitectura en robots físicos, **excluyendo** todos los componentes de simulación.

**Contenido**:
- **`agents/`**: Workflows puros de LangGraph sin dependencias de simulación
- **`ros2_integration/`**: Módulos que adaptan los workflows para funcionar con ROS 2 en robots reales
- **`ros2_controllers/`**: Controladores ROS 2 listos para desplegar en robots físicos
- **`docs/`**: Manual completo con especificaciones de hardware y software

**Cuándo usar**: Cuando quieras implementar la arquitectura en robots físicos reales. Esta carpeta contiene solo el código necesario, sin simuladores ni archivos de testing.

**Uso**: Copia esta carpeta completa a tu robot y sigue el manual de implementación.

#### 🔧 `MultiAgent/` - Desarrollo y Testing
**Propósito**: Código fuente original de los agentes LangGraph para desarrollo, testing y experimentación.

**Contenido**:
- Agentes LangGraph puros (sin integración ROS 2)
- Scripts de prueba y desarrollo (`video_sim.py`)
- Entorno virtual Python con dependencias
- Archivos de configuración y documentación

**Cuándo usar**: Para desarrollar nuevos workflows, probar cambios en los agentes, o experimentar con diferentes configuraciones sin necesidad de ROS 2.

**Uso**: Para desarrollar, probar y modificar los workflows de los agentes antes de integrarlos con ROS 2.

#### 🎮 `robotic-ai-agents/` - Simulador ROS 2
**Propósito**: Simulador completo MicroSim para probar la arquitectura sin robots físicos.

**Contenido**:
- **`microsim/`**: Motor de simulación determinístico en tiempo real
  - Modelos cinemáticos de drone y rover
  - Renderizado de cámara con colores semánticos
  - Sensores simulados (GPS, cámara, rango)
- **`scripts/`**: Controladores adaptados para el simulador
- **`multiagent/`**: Integración LangGraph específica para simulación
- **`scenarios/`**: Escenarios YAML configurables (víctimas, obstáculos, robots)
- Scripts `.bat` para iniciar el sistema completo

**Cuándo usar**: Para probar y validar la arquitectura completa antes de implementarla en robots reales, o para desarrollo continuo de nuevas funcionalidades.

**Uso**: Para probar, validar y desarrollar la arquitectura en un entorno simulado antes de desplegar en robots reales.

#### 📚 `docs/` - Documentación General
**Propósito**: Documentación compartida del proyecto.

**Contenido**:
- Manual de implementación para robots reales
- Otra documentación técnica general

## 🚀 Inicio Rápido

### Prerrequisitos

- **Python 3.8+**
- **ROS 2 Humble** (para el simulador o robots reales)
- **OpenAI API Key** (para GPT-4o/GPT-5)
- **Git**

### Opción 1: Simulación (Recomendado para empezar)

Ideal para probar y desarrollar la arquitectura sin robots físicos.

1. **Clonar el repositorio**:
   ```bash
   git clone https://github.com/renzoBC20/Disaster-multiagent-architecture-project.git
   cd Disaster-multiagent-architecture-project
   ```

2. **Configurar el entorno MultiAgent**:
   ```bash
   cd MultiAgent
   python -m venv venv
   venv\Scripts\activate  # Windows
   # source venv/bin/activate  # Linux/Mac
   pip install -r requirements.txt
   ```

3. **Configurar API Key de OpenAI**:
   ```bash
   cd MultiAgent
   cp config.example .env  # Linux/Mac
   copy config.example .env  # Windows
   # Editar .env y agregar: OPENAI_API_KEY=tu_api_key_aqui
   ```

4. **Compilar el simulador ROS 2**:
   ```bash
   cd robotic-ai-agents/simulator/microsim
   cmd /c COMPILAR.bat  # Windows
   # colcon build  # Linux/Mac
   ```

5. **Iniciar el sistema completo**:
   ```bash
   cd robotic-ai-agents/simulator/microsim
   cmd /c INICIAR_SISTEMA_COMPLETO.bat  # Windows
   # Ver docs/ para Linux/Mac
   ```

### Opción 2: Robots Reales

Para implementar en robots físicos, usar la carpeta `architecture/`.

1. **Copiar la carpeta architecture a tu robot**:
   ```bash
   # En tu robot (Linux con ROS 2)
   cp -r architecture /path/to/your/ros2_workspace/src/
   ```

2. **Seguir el manual de implementación**:
   Ver [`architecture/docs/MANUAL_IMPLEMENTACION.md`](architecture/docs/MANUAL_IMPLEMENTACION.md)

3. **Instalar dependencias**:
   ```bash
   cd architecture
   pip install -r requirements.txt
   cp config.example .env
   # Editar .env con tu API key
   ```

4. **Compilar en el workspace ROS 2**:
   ```bash
   cd /path/to/your/ros2_workspace
   colcon build --packages-select architecture
   source install/setup.bash
   ```

5. **Ejecutar los controladores**:
   ```bash
   # Terminal 1: UAV
   ros2 run architecture uav_langgraph_controller
   
   # Terminal 2: UGV
   ros2 run architecture ugv_langgraph_controller
   ```

## 🔧 Características Principales

### Sistema de Simulación (MicroSim)
- ✅ Simulación determinística en tiempo real (60 Hz)
- ✅ Modelos cinemáticos precisos para drone y rover
- ✅ Sensores realistas (GPS, cámara, rango)
- ✅ Modelo de comunicación por radio
- ✅ Interfaz ROS 2 completa

### Agente UAV
- ✅ Análisis de video en tiempo real
- ✅ Identificación de víctimas con GPT-4o Vision
  - Víctimas críticas (rojo)
  - Víctimas heridas (naranja)
  - Víctimas estables (verde)
- ✅ Detección de obstáculos
  - Edificios (marrón)
  - Escombros (magenta)
  - Árboles (amarillo)
  - Vehículos (gris)
- ✅ Planificación optimizada de rutas
- ✅ Generación de briefings estructurados

### Agente UGV
- ✅ Recepción y procesamiento de misiones
- ✅ Sistema de detección de colisiones
- ✅ Replanificación automática con GPT
- ✅ Ejecución de rescate paso a paso
- ✅ Navegación con evasión de obstáculos

## 🎨 Sistema de Colores

### Víctimas (Círculos)
- 🔴 **Rojo** (#FF0000): Víctima crítica - necesita rescate inmediato
- 🟠 **Naranja** (#FF6700): Víctima herida - necesita atención médica
- 🟢 **Verde Oscuro** (#006400): Víctima segura - estable

### Obstáculos (Cuadrados)
- 🟤 **Marrón** (#8B4513): Edificios/estructuras
- 🟣 **Magenta** (#FF00FF): Escombros
- 🟡 **Amarillo** (#C8AA3C): Árboles
- ⚫ **Gris Oscuro** (#404040): Vehículos

### Robots
- 🔵 **Azul** (#0000FF): Rover (UGV)
- ⚪ **Blanco**: Drone (UAV)

## 📊 Flujo de Trabajo del Sistema

### Flujo Operativo (Ejecución)

1. **Inicialización**: El simulador crea el mundo con víctimas y obstáculos
2. **Reconocimiento Aéreo**: El UAV vuela y captura imágenes del área
3. **Análisis con IA**: GPT-4o/GPT-5 identifica víctimas y obstáculos en cada frame
4. **Planificación**: El UAV genera rutas optimizadas visitando todas las víctimas
5. **Briefing de Misión**: Se compila un mensaje estructurado para el UGV
6. **Ejecución Terrestre**: El UGV recibe la misión y ejecuta el rescate
7. **Detección de Colisiones**: El UGV evita obstáculos y replanifica si es necesario
8. **Rescate**: El UGV visita cada víctima según la prioridad

### Flujo de Desarrollo (Organización del Proyecto)

```
┌─────────────────────────────────────────────────────────────┐
│  MultiAgent/                                                │
│  └─ Desarrollo de workflows LangGraph                      │
│     └─ Testing con video estático                          │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│  robotic-ai-agents/simulator/microsim/                     │
│  └─ Integración con ROS 2                                  │
│     └─ Validación en simulación                            │
└────────────────────┬────────────────────────────────────────┘
                     │
                     ▼
┌─────────────────────────────────────────────────────────────┐
│  architecture/                                              │
│  └─ Código limpio sin simulación                          │
│     └─ Listo para robots reales                            │
└─────────────────────────────────────────────────────────────┘
```

**Proceso de desarrollo recomendado**:
1. Desarrollar/modificar workflows en `MultiAgent/`
2. Integrar y probar en `robotic-ai-agents/` (simulación)
3. Validar funcionamiento completo
4. Copiar código validado a `architecture/` para robots reales

## 🔬 Tecnologías Utilizadas

- **ROS 2 Humble**: Framework de robótica
- **LangGraph**: Framework para aplicaciones multi-agente con estado
- **OpenAI GPT-4o**: Modelo de lenguaje multimodal para análisis visual
- **OpenCV**: Procesamiento de imágenes y video
- **NumPy**: Cálculos numéricos
- **Python 3.8+**: Lenguaje principal

## 📝 Documentación

### 📘 Documentación Principal

- **[Manual de Implementación para Robots Reales](architecture/docs/MANUAL_IMPLEMENTACION.md)** ⭐
  - Especificaciones completas de hardware y software
  - Guía paso a paso para implementación en robots físicos
  - Requisitos de sensores, computación y comunicación

- **[README de Arquitectura](architecture/README.md)**
  - Guía de la carpeta `architecture/`
  - Estructura y uso de archivos para robots reales

### 🔧 Documentación de Desarrollo

- [Documentación MultiAgent](MultiAgent/docs/README.md) - Desarrollo de agentes LangGraph
- [Documentación MicroSim](robotic-ai-agents/simulator/microsim/docs/) - Simulador ROS 2
- [Guía de Instalación](robotic-ai-agents/simulator/microsim/INSTRUCCIONES_INSTALACION.md) - Setup del simulador
- [Arquitectura LangGraph](robotic-ai-agents/simulator/microsim/ARQUITECTURA_LANGGRAPH.md) - Diseño de workflows

## 🤝 Contribuciones

Las contribuciones son bienvenidas. Por favor:

1. Fork el repositorio
2. Crea una rama para tu feature (`git checkout -b feature/AmazingFeature`)
3. Commit tus cambios (`git commit -m 'Add some AmazingFeature'`)
4. Push a la rama (`git push origin feature/AmazingFeature`)
5. Abre un Pull Request

## 📄 Licencia

Este proyecto está licenciado bajo la Licencia MIT - ver el archivo [LICENSE](MultiAgent/LICENSE) para más detalles.

## 👤 Autor

**Renzo BC20**

- GitHub: [@renzoBC20](https://github.com/renzoBC20)

## 🙏 Agradecimientos

- [RoboticAIAgents](https://github.com/RoboticAIAgents) por la arquitectura base
- OpenAI por GPT-4o
- Comunidad ROS 2
- LangChain por LangGraph

---

⭐ Si este proyecto te resulta útil, considera darle una estrella en GitHub!

