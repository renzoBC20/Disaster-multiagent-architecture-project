# 🚀 Plan de Trabajo: Integración MultiAgent UAV-UGV con MicroSim

## 📋 Objetivo General

Integrar el sistema MultiAgent UAV-UGV (basado en LangGraph + OpenAI) con el simulador ROS 2 MicroSim para crear un sistema completo de rescate autónomo.

---

## 🎯 Fase 1: Análisis y Preparación

### ✅ Estado Actual
- **MultiAgent**: Sistema funcionando con LangGraph, análisis de video estático
- **MicroSim**: Simulador ROS 2 con controlador básico funcionando
- **Entorno**: ROS 2 Humble instalado con conda, workspace configurado

### 📝 Tareas
1. **Analizar arquitectura actual** ✅
   - UAV Agent: LangGraph con funciones de análisis
   - UGV Agent: Sistema de detección de colisiones
   - MicroSim: Topics ROS 2 disponibles

2. **Definir interfaz de comunicación**
   - ROS 2 Topics para comunicación inter-agente
   - Mensajes personalizados para misiones
   - Servicios para sincronización

---

## 🔧 Fase 2: Integración UAV con MicroSim

### Tarea 2.1: Crear Bridge LangGraph ↔ ROS 2
**Archivo**: `scripts/uav_ros2_bridge.py`

**Funcionalidad**:
- Conectar LangGraph State con ROS 2 Node
- Suscribirse a `/drone/camera/image_raw` (en lugar de video estático)
- Convertir imágenes ROS 2 → OpenCV → Base64 para GPT
- Publicar decisiones del agente como comandos de velocidad

**Implementación**:
```python
class UAVROS2Bridge(AutonomousDroneController):
    def __init__(self):
        super().__init__()
        # Inicializar LangGraph workflow
        self.uav_workflow = create_uav_workflow()
        self.state = initialize_uav_state()
        
    def camera_callback(self, msg):
        # Convertir Image ROS 2 → OpenCV → Base64
        # Actualizar state de LangGraph
        # Ejecutar workflow
        pass
```

### Tarea 2.2: Adaptar Análisis de Víctimas
- Convertir coordenadas de imagen → coordenadas del mundo de MicroSim
- Mapear detecciones de GPT a world features del simulador
- Integrar con sistema de waypoints de MicroSim

### Tarea 2.3: Implementar Control Inteligente
- Usar decisiones de LangGraph para generar comandos de velocidad
- Mantener compatibilidad con controlador básico
- Agregar logging de decisiones del agente

---

## 🤖 Fase 3: Integración UGV con MicroSim

### Tarea 3.1: Crear Controlador UGV ROS 2
**Archivo**: `scripts/ugv_ros2_controller.py`

**Funcionalidad**:
- Suscribirse a topics de misión del UAV
- Implementar detección de colisiones con sensores de MicroSim
- Publicar comandos de velocidad al rover

### Tarea 3.2: Sistema de Detección de Colisiones
- Usar `/rover/range` sensor de MicroSim
- Adaptar ProximitySensor a datos ROS 2
- Integrar replanificación con GPT cuando detecte obstáculos

### Tarea 3.3: Comunicación Inter-Agente
- Crear custom message types para misiones
- Implementar publisher/subscriber entre UAV y UGV
- Reemplazar comunicación vía archivo JSON por ROS 2 topics

---

## 🌉 Fase 4: Comunicación Multi-Agente

### Tarea 4.1: Definir Mensajes Personalizados
**Archivo**: `msg/MissionBrief.msg`

```msg
string from_agent
string to_agent
string message_type
Victim[] victims
Obstacle[] obstacles
RoutePoint[] route
```

### Tarea 4.2: Topics de Comunicación
- `/uav/mission_brief`: UAV publica misión para UGV
- `/ugv/status`: UGV publica estado al UAV
- `/ugv/collision_alert`: UGV alerta al UAV sobre obstáculos

### Tarea 4.3: Servicios de Sincronización
- `/uav/request_status`: UGV solicita estado del UAV
- `/sim/reset`: Resetear simulación coordinada

---

## 🎨 Fase 5: Visualización y Monitoreo

### Tarea 5.1: Visualización Integrada
- Extender `viz_2d.py` para mostrar:
  - Decisiones del agente UAV
  - Rutas planificadas
  - Detecciones de víctimas
  - Alertas de colisión UGV

### Tarea 5.2: Logging y Métricas
- Registrar decisiones del agente
- Métricas de performance (tiempo de respuesta, precisión)
- Exportar logs para análisis

---

## 📊 Fase 6: Testing y Validación

### Tarea 6.1: Tests Unitarios
- Test de conversión imagen → coordenadas mundo
- Test de detección de colisiones
- Test de comunicación inter-agente

### Tarea 6.2: Tests de Integración
- Test completo UAV → UGV → Rescate
- Test de replanificación ante obstáculos
- Test de múltiples escenarios

### Tarea 6.3: Validación con Escenarios
- Escenario simple (1 víctima)
- Escenario complejo (múltiples víctimas + obstáculos)
- Escenario dinámico (obstáculos que aparecen)

---

## 📁 Estructura de Archivos Propuesta

```
robotic-ai-agents/simulator/microsim/
├── scripts/
│   ├── autonomous_drone_controller.py    # Controlador básico (existente)
│   ├── uav_ros2_bridge.py               # 🆕 Bridge LangGraph → ROS 2
│   ├── uav_ai_controller.py             # 🆕 Controlador UAV inteligente
│   ├── ugv_ros2_controller.py           # 🆕 Controlador UGV inteligente
│   └── viz_2d_enhanced.py               # 🆕 Visualización mejorada
├── msg/
│   ├── MissionBrief.msg                 # 🆕 Mensaje de misión
│   ├── Victim.msg                        # 🆕 Info de víctima
│   └── Obstacle.msg                      # 🆕 Info de obstáculo
├── srv/
│   └── AgentSync.srv                     # 🆕 Servicio de sincronización
└── multiagent/
    ├── __init__.py
    ├── langgraph_integration.py          # 🆕 Funciones LangGraph adaptadas
    ├── coordinate_transforms.py          # 🆕 Transformaciones coordenadas
    └── mission_parser.py                 # 🆕 Parser de misiones
```

---

## 🚀 Próximos Pasos Inmediatos

### 1. Crear estructura base
- [x] Crear directorio `multiagent/`
- [x] Crear `uav_ai_controller.py` con estructura básica
- [ ] Definir mensajes personalizados

### 2. Integración básica UAV
- [ ] Conectar cámara de MicroSim con análisis de GPT
- [ ] Mapear coordenadas imagen → mundo
- [ ] Publicar comandos de velocidad desde decisiones del agente

### 3. Integración básica UGV
- [ ] Crear controlador UGV ROS 2
- [ ] Implementar suscripción a misiones del UAV
- [ ] Integrar detección de colisiones con sensores ROS 2

---

## 📝 Notas de Implementación

### Conversión de Coordenadas
- Imagen: (0,0) = esquina superior izquierda, pixels
- Mundo MicroSim: (0,0) = origen, metros
- **Conversión necesaria**: `world_x = (pixel_x / img_width) * world_size`

### Timing
- Control loop UAV: 10 Hz (100ms por decisión)
- Análisis GPT: ~2-5 segundos por frame
- **Solución**: Procesar frames asíncronamente, usar última decisión

### Sincronización
- UAV procesa frames y genera misiones
- UGV espera misión antes de moverse
- **Mecanismo**: ROS 2 service para confirmación de misión recibida

---

## ✅ Criterios de Éxito

1. ✅ UAV analiza cámara de MicroSim en tiempo real
2. ✅ UGV recibe misión vía ROS 2 topic
3. ✅ Sistema de detección de colisiones funciona
4. ✅ Replanificación automática ante obstáculos
5. ✅ Visualización muestra decisiones del agente
6. ✅ Sistema completo funcional end-to-end

---

**Fecha de inicio**: 2025-10-29
**Estado**: 🟢 Listo para comenzar

