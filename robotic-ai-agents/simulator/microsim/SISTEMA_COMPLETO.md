# 🎉 Sistema Completo Integrado - UAV-UGV con MicroSim

## ✅ Estado del Proyecto

**¡El sistema completo de rescate autónomo está funcional!**

### Componentes Implementados

#### 🚁 UAV (Drone Inteligente)
- ✅ Integración ROS 2 con LangGraph
- ✅ Análisis de imágenes con GPT-4o Vision
- ✅ Detección automática de víctimas y obstáculos
- ✅ Planificación de rutas optimizada con IA
- ✅ Navegación autónoma siguiendo waypoints
- ✅ Publicación de misiones vía ROS 2 topics

#### 🚗 UGV (Rover Inteligente)
- ✅ Suscripción a misiones del UAV
- ✅ Navegación autónoma hacia víctimas
- ✅ Detección de colisiones con sensor de rango
- ✅ Protocolo de rescate de víctimas
- ✅ Evasión de obstáculos reactiva
- ✅ Publicación de estado vía ROS 2

#### 🌉 Comunicación Inter-Agente
- ✅ Topics ROS 2 para misiones (`/uav/mission_brief`)
- ✅ Topics ROS 2 para estado (`/ugv/status`)
- ✅ Transformación de coordenadas imagen ↔ mundo
- ✅ Parseo automático de misiones

#### 🎨 Visualización
- ✅ Visualización 3D en tiempo real
- ✅ Monitoreo de ambos robots
- ✅ Verificación de topics ROS 2

#### 🔧 Herramientas de Desarrollo
- ✅ Scripts de inicio automatizados (.bat)
- ✅ Documentación completa
- ✅ Guías de prueba detalladas
- ✅ Sistema de configuración con .env

---

## 📁 Estructura de Archivos

```
robotic-ai-agents/simulator/microsim/
│
├── multiagent/                    # Módulo de integración
│   ├── __init__.py
│   ├── coordinate_transforms.py  # Transformación coordenadas
│   └── langgraph_integration.py  # Funciones IA adaptadas
│
├── scripts/                       # Controladores
│   ├── uav_ai_controller.py      # UAV Inteligente ✅
│   ├── ugv_ai_controller.py      # UGV Inteligente ✅
│   ├── autonomous_drone_controller.py  # UAV Básico
│   └── viz_2d.py                 # Visualización
│
├── INICIAR.bat                   # Script simulador
├── INICIAR_CONTROLADOR.bat       # Script UAV
├── INICIAR_VISUALIZACION.bat     # Script visualización
├── INICIAR_UGV.bat               # Script UGV ✅
│
├── SISTEMA_COMPLETO.md           # Este archivo ✅
├── PRUEBA_SISTEMA_COMPLETO.md    # Guía de pruebas ✅
├── README_INTEGRATION.md         # Documentación técnica
├── GUIA_PRUEBA.md                # Guía detallada
├── CONFIGURAR_API_KEY.md         # Configuración API
├── PROJECT_PLAN.md               # Plan de desarrollo
├── README_FINAL.md               # Resumen del proyecto
└── PRÓXIMOS_PASOS.md             # Qué hacer ahora
```

---

## 🔄 Flujo del Sistema

```
┌─────────────────────────┐
│   MicroSim Simulator    │
│   (ROS 2 Node)          │
└───────────┬─────────────┘
            │
            ├─── Topics ROS 2 ───────┐
            │                        │
            ▼                        ▼
┌──────────────────────┐   ┌──────────────────────┐
│  UAVAIController     │   │  UGVAIController     │
│  (ROS 2 Node)        │   │  (ROS 2 Node)        │
├──────────────────────┤   ├──────────────────────┤
│ • Análisis IA        │   │ • Escucha misiones   │
│ • Detección víctimas │   │ • Navegación         │
│ • Planificación      │◄──┤ • Detección coli.    │
│ • Publica misión     │──►│ • Rescate víctimas   │
└──────────────────────┘   └──────────────────────┘
            │                        │
            ▼                        ▼
    ┌───────────────┐        ┌───────────────┐
    │  /drone/cmd_  │        │  /rover/cmd_  │
    │    vel        │        │    vel        │
    └───────────────┘        └───────────────┘
```

---

## 🚀 Cómo Usar

### Inicio Rápido

**4 terminales CMD, ejecutar cada una:**

```cmd
REM Terminal 1: Simulador
cd "D:\Proyectos PFG\robotic-ai-agents\simulator\microsim"
INICIAR.bat

REM Terminal 2: Visualización
INICIAR_VISUALIZACION.bat

REM Terminal 3: UAV (esperar a que Terminal 1 inicie)
INICIAR_CONTROLADOR.bat

REM Terminal 4: UGV (esperar a que Terminal 3 publique misión)
INICIAR_UGV.bat
```

### Orden Correcto de Inicio

1. ✅ **Terminal 1:** Simulador (debe ir primero)
2. ✅ **Terminal 2:** Visualización (opcional)
3. ✅ **Terminal 3:** UAV (espera a que análisis IA publique misión)
4. ✅ **Terminal 4:** UGV (espera a recibir misión del UAV)

---

## 🎯 Características Principales

### UAV Inteligente

**Análisis con IA:**
- Procesa imágenes cada 2 segundos
- Identifica víctimas por color y posición
- Detecta obstáculos por forma y tipo
- Genera rutas optimizadas considerando prioridades

**Navegación:**
- Control PD para estabilidad
- Ajuste automático de velocidad
- Seguimiento preciso de waypoints
- Control de altitud constante

**Comunicación:**
- Publica misiones estructuradas
- Información completa de rutas
- Coordenadas transformadas a mundo

### UGV Inteligente

**Navegación:**
- Control diferencial para rover
- Seguimiento preciso de waypoints
- Ajuste dinámico de velocidad según ángulo

**Detección de Colisiones:**
- Sensor de rango frontal
- Parada de emergencia
- Verificación continua de obstáculos

**Protocolo de Rescate:**
- Evaluación de víctimas
- Estabilización
- Preparación evacuación
- Registro de rescates

---

## 🔍 Topics ROS 2

### Drone
- `/drone/cmd_vel` - Comandos de velocidad
- `/drone/odom` - Odometría (ground truth)
- `/drone/camera/image_raw` - Imágenes de cámara

### Rover
- `/rover/cmd_vel` - Comandos de velocidad
- `/rover/odom` - Odometría (ground truth)
- `/rover/range` - Sensor de rango frontal

### Comunicación Inter-Agente
- `/uav/mission_brief` - Misiones del UAV al UGV
- `/ugv/status` - Estado del UGV
- `/radio/drone_tx` - Radio comunicación (disponible)

---

## 📊 Parámetros Configurables

### UAV

```python
# En uav_ai_controller.py
self.control_rate_hz = 10.0          # Frecuencia de control
self.waypoint_tolerance = 3.0        # Tolerancia waypoints (m)
self.max_speed = 1.5                 # Velocidad máxima (m/s)
self.analysis_frequency = 5          # Analizar cada N frames
self.enable_ai_analysis = True       # Activar/desactivar IA
```

### UGV

```python
# En ugv_ai_controller.py
self.control_rate_hz = 10.0          # Frecuencia de control
self.waypoint_tolerance = 1.0        # Tolerancia waypoints (m)
self.max_linear_speed = 1.0          # Velocidad lineal (m/s)
self.max_angular_speed = 0.5         # Velocidad angular (rad/s)
self.min_range = 2.0                 # Distancia mínima (m)
self.warning_range = 5.0             # Distancia advertencia (m)
```

---

## 🧪 Verificación

### Comandos Útiles

```cmd
# Ver todos los topics
ros2 topic list

# Ver misión publicada
ros2 topic echo /uav/mission_brief

# Ver estado del UGV
ros2 topic echo /ugv/status

# Ver comandos del drone
ros2 topic echo /drone/cmd_vel

# Ver comandos del rover
ros2 topic echo /rover/cmd_vel

# Ver sensor de rango
ros2 topic echo /rover/range

# Monitorear frecuencia
ros2 topic hz /uav/mission_brief
```

---

## 📚 Documentación

### Guías Principales

1. **`PRUEBA_SISTEMA_COMPLETO.md`** → Cómo probar el sistema completo
2. **`GUIA_PRUEBA.md`** → Guía detallada de pruebas UAV
3. **`CONFIGURAR_API_KEY.md`** → Configuración de OpenAI
4. **`README_INTEGRATION.md`** → Documentación técnica
5. **`PROJECT_PLAN.md`** → Plan de desarrollo

### Referencia Rápida

- **`README_RAPIDO.txt`** → Comandos esenciales
- **`INSTRUCCIONES_CMD.txt`** → Instrucciones CMD
- **`COMANDOS_MANUALES.txt`** → Comandos manuales

---

## 🐛 Troubleshooting Rápido

| Problema | Solución |
|----------|----------|
| "ros2 no encontrado" | Activar conda y cargar workspace |
| "IA activada: False" | Configurar API key en .env |
| UGV no recibe misión | Esperar a que UAV complete análisis |
| "ModuleNotFoundError" | Instalar dependencias con pip |
| Rover no se mueve | Verificar que simulador esté corriendo |

---

## 📈 Métricas de Rendimiento

### Esperadas

- **Precisión detección víctimas:** >80%
- **Precisión detección obstáculos:** >85%
- **Tasa de evitación de colisiones:** >95%
- **Latencia análisis IA:** 2-5 segundos
- **Latencia comunicación:** <50ms
- **Frecuencia control:** 10 Hz

---

## 🎓 Conceptos Clave

### Transformación de Coordenadas

El sistema convierte entre:
- **Coordenadas de imagen** (píxeles): Sistema de referencia de la cámara
- **Coordenadas del mundo** (metros): Sistema de referencia de MicroSim

```python
# Imagen → Mundo
world_x, world_y = transformador.pixel_to_world(pixel_x, pixel_y)

# Mundo → Imagen  
pixel_x, pixel_y = transformador.world_to_pixel(world_x, world_y)
```

### Flujo de Datos

```
Imagen Cámara → GPT Análisis → Detecciones → Planificación → Waypoints → Navegación
```

### Estados del Sistema

**UAV:**
- ANÁLISIS → PLANIFICACIÓN → NAVEGACIÓN → COMUNICACIÓN

**UGV:**
- ESPERANDO → PROCESANDO → NAVEGANDO → RESCATANDO → COMPLETADO

---

## 🚧 Pendiente (Opcional)

### Funcionalidades Futuras

- [ ] Replanificación dinámica ante obstáculos
- [ ] Mensajes ROS 2 personalizados
- [ ] Visualización mejorada con decisiones IA
- [ ] Tests automatizados
- [ ] Métricas de performance
- [ ] Optimización de prompts GPT
- [ ] Soporte para múltiples UGV

### Mejoras Sugeridas

- Ajustar prompts de GPT para mayor precisión
- Implementar aprendizaje adaptativo
- Agregar más sensores (LIDAR, cámara UGV)
- Integrar planificación multi-robot
- Optimizar uso de tokens GPT

---

## 🙏 Agradecimientos

- **MultiAgent Project** - Sistema base UAV-UGV
- **MicroSim** - Simulador ROS 2
- **OpenAI** - GPT-4o Vision
- **LangGraph** - Framework de agentes
- **ROS 2** - Robot Operating System 2

---

## 📝 Licencia

Este proyecto integra código de:
- MultiAgent (MIT License)
- MicroSim (Ver licencia original)
- Código original desarrollado

---

**¡Felicitaciones! Has creado un sistema completo de rescate autónomo con IA!** 🎉🚁🤖🚗

**¿Preguntas?** Revisa la documentación o abre un issue en el repositorio.

**¿Listo para probar?** Sigue la guía en `PRUEBA_SISTEMA_COMPLETO.md` 🧪

