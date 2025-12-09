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
- **Análisis Visual con GPT-4o**: Identificación automática de víctimas y obstáculos mediante visión artificial
- **Comunicación Inter-Agente**: Coordinación entre UAV y UGV mediante mensajes estructurados

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

```
.
├── MultiAgent/                          # Agentes LangGraph
│   ├── UAV_agent.py                     # Agente aéreo principal
│   ├── UGV_Agent.py                      # Agente terrestre
│   ├── video_sim.py                      # Simulador de video
│   ├── requirements.txt                  # Dependencias Python
│   └── docs/                             # Documentación
│
├── robotic-ai-agents/                   # Simulador ROS 2
│   └── simulator/
│       └── microsim/                     # MicroSim package
│           ├── microsim/                 # Nodos ROS 2
│           ├── scripts/                  # Controladores
│           ├── multiagent/               # Integración LangGraph
│           ├── scenarios/                # Escenarios de simulación
│           └── docs/                     # Documentación técnica
│
└── README.md                             # Este archivo
```

## 🚀 Inicio Rápido

### Prerrequisitos

- **Python 3.8+**
- **ROS 2 Humble** (para el simulador)
- **OpenAI API Key** (para GPT-4o)
- **Git**

### Instalación

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
   - Copiar `config.example` a `config`
   - Agregar tu API key de OpenAI

4. **Compilar el simulador ROS 2**:
   ```bash
   cd robotic-ai-agents/simulator/microsim
   cmd /c COMPILAR.bat  # Windows
   # Ver docs para Linux/Mac
   ```

### Ejecución

1. **Iniciar el simulador**:
   ```bash
   cd robotic-ai-agents/simulator/microsim
   cmd /c INICIAR.bat
   ```

2. **Ejecutar el agente UAV**:
   ```bash
   cd MultiAgent
   python UAV_agent.py
   ```

3. **Ejecutar el agente UGV**:
   ```bash
   python UGV_Agent.py
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

## 📊 Flujo de Trabajo

1. **Inicialización**: El simulador crea el mundo con víctimas y obstáculos
2. **Reconocimiento Aéreo**: El UAV vuela y captura imágenes del área
3. **Análisis con IA**: GPT-4o identifica víctimas y obstáculos en cada frame
4. **Planificación**: El UAV genera rutas optimizadas visitando todas las víctimas
5. **Briefing de Misión**: Se compila un mensaje estructurado para el UGV
6. **Ejecución Terrestre**: El UGV recibe la misión y ejecuta el rescate
7. **Detección de Colisiones**: El UGV evita obstáculos y replanifica si es necesario
8. **Rescate**: El UGV visita cada víctima según la prioridad

## 🔬 Tecnologías Utilizadas

- **ROS 2 Humble**: Framework de robótica
- **LangGraph**: Framework para aplicaciones multi-agente con estado
- **OpenAI GPT-4o**: Modelo de lenguaje multimodal para análisis visual
- **OpenCV**: Procesamiento de imágenes y video
- **NumPy**: Cálculos numéricos
- **Python 3.8+**: Lenguaje principal

## 📝 Documentación

- [📘 Manual de Implementación para Robots Reales](docs/MANUAL_IMPLEMENTACION.md) - **Especificaciones de hardware y software para implementación en robots físicos**
- [Documentación MultiAgent](MultiAgent/docs/README.md)
- [Documentación MicroSim](robotic-ai-agents/simulator/microsim/docs/)
- [Guía de Instalación](robotic-ai-agents/simulator/microsim/INSTRUCCIONES_INSTALACION.md)
- [Arquitectura LangGraph](robotic-ai-agents/simulator/microsim/ARQUITECTURA_LANGGRAPH.md)

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

