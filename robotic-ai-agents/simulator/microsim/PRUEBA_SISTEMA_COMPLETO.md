# 🧪 Prueba del Sistema Completo UAV-UGV

## 🎯 Sistema Integrado

Ahora tienes un sistema completo de rescate autónomo con:
- ✅ UAV que analiza el entorno con IA
- ✅ UAV que genera rutas optimizadas
- ✅ UAV que publica misiones al UGV
- ✅ UGV que escucha y ejecuta misiones
- ✅ UGV que detecta colisiones
- ✅ UGV que rescata víctimas

---

## 📋 Requisitos Previos

1. ✅ Archivo `.env` configurado con API key de OpenAI
2. ✅ Workspace ROS 2 compilado
3. ✅ Dependencias instaladas

---

## 🚀 Ejecutar el Sistema Completo

Necesitas **4 terminales** abiertas simultáneamente:

### Terminal 1: Simulador MicroSim

```cmd
cd "D:\Proyectos PFG\robotic-ai-agents\simulator\microsim"
INICIAR.bat
```

**O manualmente:**
```cmd
call "%LOCALAPPDATA%\miniforge3\condabin\conda.bat" activate ros2_humble
cd "D:\Proyectos PFG\robotic-ai-agents\simulator\microsim"
call install\setup.bat
ros2 run microsim microsim_node
```

**Qué esperar:**
- El simulador carga el escenario
- Aparece el drone y el rover en el mundo

---

### Terminal 2: Visualización (Opcional pero Recomendado)

```cmd
cd "D:\Proyectos PFG\robotic-ai-agents\simulator\microsim"
INICIAR_VISUALIZACION.bat
```

**O manualmente:**
```cmd
call "%LOCALAPPDATA%\miniforge3\condabin\conda.bat" activate ros2_humble
cd "D:\Proyectos PFG\robotic-ai-agents\simulator\microsim"
python scripts\viz_2d.py
```

**Qué esperar:**
- Se abre ventana 3D con visualización
- Puedes ver drone y rover desde arriba

---

### Terminal 3: Controlador UAV Inteligente

```cmd
cd "D:\Proyectos PFG\robotic-ai-agents\simulator\microsim"
INICIAR_CONTROLADOR.bat
```

**O manualmente:**
```cmd
call "%LOCALAPPDATA%\miniforge3\condabin\conda.bat" activate ros2_humble
cd "D:\Proyectos PFG\robotic-ai-agents\simulator\microsim"
call install\setup.bat
python scripts\uav_ai_controller.py
```

**Qué esperar:**
- Inicialización del controlador UAV
- Mensajes de análisis de IA cada 2 segundos
- El drone comienza a moverse y analizar el entorno
- Publicación de misiones al UGV

---

### Terminal 4: Controlador UGV Inteligente

**⚠️ IMPORTANTE:** Espera a que el UAV haya publicado una misión antes de iniciar el UGV.

```cmd
cd "D:\Proyectos PFG\robotic-ai-agents\simulator\microsim"
INICIAR_UGV.bat
```

**O manualmente:**
```cmd
call "%LOCALAPPDATA%\miniforge3\condabin\conda.bat" activate ros2_humble
cd "D:\Proyectos PFG\robotic-ai-agents\simulator\microsim"
call install\setup.bat
python scripts\ugv_ai_controller.py
```

**Qué esperar:**
- Inicialización del controlador UGV
- Mensaje de "Esperando misión del UAV..."
- Cuando llegue la misión: el UGV comienza a moverse hacia las víctimas
- El UGV detecta colisiones y rescata víctimas

---

## 🔍 Monitorear el Sistema

### Ver Topics ROS 2

En una terminal adicional:

```cmd
# Ver todos los topics
ros2 topic list

# Ver mensajes de misión
ros2 topic echo /uav/mission_brief

# Ver estado del UGV
ros2 topic echo /ugv/status

# Ver comandos del drone
ros2 topic echo /drone/cmd_vel

# Ver comandos del rover
ros2 topic echo /rover/cmd_vel

# Ver sensor de rango
ros2 topic echo /rover/range
```

---

## 📊 Flujo de Funcionamiento

### Fase 1: Análisis UAV ⬇️

1. **Captura**: MicroSim publica imágenes de la cámara del drone
2. **Análisis IA**: El UAV analiza cada frame con GPT
3. **Detección**: Identifica víctimas y obstáculos
4. **Planificación**: Genera ruta optimizada
5. **Navegación**: El drone sigue la ruta

### Fase 2: Publicación de Misión ⬇️

6. **Mensaje**: UAV publica misión en `/uav/mission_brief`
7. **Contenido**: Rutas, víctimas, obstáculos, coordenadas

### Fase 3: Ejecución UGV ⬇️

8. **Recepción**: UGV recibe la misión
9. **Navegación**: Se mueve hacia las víctimas
10. **Detección**: Usa sensor de rango para evitar colisiones
11. **Rescate**: Ejecuta protocolo de atención a víctimas
12. **Confirmación**: Registra víctimas rescatadas

---

## ✅ Verificación de Funcionamiento

### Checklist de Éxito

#### Terminal 1 (Simulador)
- [ ] Simulador iniciado sin errores
- [ ] Mundo cargado correctamente
- [ ] Drone y rover visibles

#### Terminal 2 (Visualización)
- [ ] Ventana 3D abierta
- [ ] Mundo visible desde arriba
- [ ] Robot moviéndose en tiempo real

#### Terminal 3 (UAV)
- [ ] Controlador iniciado
- [ ] "IA activada: True"
- [ ] Mensajes de análisis cada 2 segundos
- [ ] Detección de víctimas/obstáculos
- [ ] Drone moviéndose siguiendo ruta

#### Terminal 4 (UGV)
- [ ] Controlador iniciado
- [ ] "Esperando misión del UAV..."
- [ ] Misión recibida y procesada
- [ ] Waypoints cargados
- [ ] UGV moviéndose hacia víctimas
- [ ] Protocolos de rescate ejecutados

---

## 🎬 Salida Esperada

### Terminal UAV

```
[INFO] [uav_ai_controller]: 🚁 UAV AI Controller inicializado!
[INFO] [uav_ai_controller]:    IA activada: True
[INFO] [uav_ai_controller]:    Listo para análisis inteligente
[INFO] [uav_ai_controller]: 🔍 Iniciando análisis de IA...
[INFO] [uav_ai_controller]: 👥 Identificando víctimas...
[INFO] [uav_ai_controller]: ✅ Detectadas 3 víctimas
[INFO] [uav_ai_controller]: 🚧 Identificando obstáculos...
[INFO] [uav_ai_controller]: ✅ Detectados 2 obstáculos
[INFO] [uav_ai_controller]: 🗺️ Planificando ruta...
[INFO] [uav_ai_controller]: ✅ Ruta planificada y actualizada
[INFO] [uav_ai_controller]: 📤 Misión publicada al UGV
```

### Terminal UGV

```
[INFO] [ugv_ai_controller]: 🚗 UGV AI Controller inicializado!
[INFO] [ugv_ai_controller]:    Esperando misión del UAV...
[INFO] [ugv_ai_controller]: 📨 Misión recibida del UAV
[INFO] [ugv_ai_controller]: ✅ 5 waypoints cargados
[INFO] [ugv_ai_controller]: 🚀 Iniciando misión de rescate
[INFO] [ugv_ai_controller]: 📍 Waypoint 1/5 alcanzado
[INFO] [ugv_ai_controller]: 🚑 Iniciando protocolo de rescate para víctima 1
[INFO] [ugv_ai_controller]: ✅ Víctima 1 rescatada! Total: 1
```

---

## 🐛 Troubleshooting

### UGV no recibe misión

**Síntomas:**
- UGV muestra "Esperando misión del UAV..." indefinidamente

**Soluciones:**
1. Verifica que el UAV esté corriendo
2. Verifica que la IA esté activada en el UAV
3. Verifica el topic: `ros2 topic echo /uav/mission_brief`
4. Espera a que el UAV complete el primer análisis (puede tomar 10-20 segundos)

### UGV no se mueve

**Síntomas:**
- Misión recibida pero el rover no se mueve

**Soluciones:**
1. Verifica que el simulador esté corriendo
2. Verifica el topic: `ros2 topic echo /rover/cmd_vel`
3. Verifica que haya waypoints en la ruta
4. Verifica que el rover no esté bloqueado

### Colisiones no detectadas

**Síntomas:**
- El rover choca con obstáculos

**Soluciones:**
1. Verifica el sensor: `ros2 topic echo /rover/range`
2. Ajusta `min_range` en el controlador UGV
3. El sensor solo detecta obstáculos adelante

### Víctimas no rescatadas

**Síntomas:**
- El rover pasa por las víctimas sin detenerse

**Soluciones:**
1. Verifica que el tipo de waypoint sea "victima"
2. Verifica las coordenadas en la visualización
3. Ajusta `waypoint_tolerance` si es necesario

---

## 📈 Métricas de Éxito

### Tiempos Esperados

- **Análisis inicial UAV**: 10-20 segundos
- **Planificación de ruta**: 5-10 segundos
- **Navegación a víctima**: 20-60 segundos (depende de la distancia)
- **Protocolo de rescate**: 5 segundos por víctima

### Indicadores de Rendimiento

- ✅ UAV detecta >80% de víctimas correctamente
- ✅ Rutas planificadas son eficientes
- ✅ UGV evita >95% de obstáculos
- ✅ Protocolos de rescate completan exitosamente
- ✅ Comunicación UAV ↔ UGV sin pérdidas

---

## 🎯 Próximos Pasos

Una vez que el sistema funcione:

1. **Ajustar parámetros** según el escenario
2. **Mejorar prompts** de GPT para más precisión
3. **Implementar replanificación** dinámica
4. **Agregar métricas** de performance
5. **Crear tests** automatizados
6. **Mejorar visualización** con decisiones IA

---

## 📚 Referencias

- `GUIA_PRUEBA.md` - Guía detallada de pruebas
- `README_INTEGRATION.md` - Documentación técnica
- `PROJECT_PLAN.md` - Plan de desarrollo

---

**¡Listo! Ahora ejecuta el sistema completo y observa la magia de la IA!** 🎉🚁🤖🚗

