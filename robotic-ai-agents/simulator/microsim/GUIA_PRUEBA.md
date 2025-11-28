# 🧪 Guía de Prueba - Controlador UAV Inteligente

## 📋 Preparación

### 1. Verificar Entorno ROS 2

Abre PowerShell y ejecuta:

```powershell
# Activar entorno conda
& "$env:LOCALAPPDATA\miniforge3\condabin\conda.bat" activate ros2_humble

# Navegar al workspace
cd "D:\Proyectos PFG\robotic-ai-agents\simulator\microsim"

# Cargar workspace ROS 2
& install\setup.bat

# Verificar que funciona
ros2 --help
```

### 2. Verificar Dependencias Python

```powershell
# Verificar dependencias básicas
python -c "import rclpy; print('✅ rclpy OK')"
python -c "import cv2; print('✅ opencv OK')"
python -c "import numpy; print('✅ numpy OK')"

# Verificar dependencias IA (opcional pero recomendado)
python -c "import langchain_openai; print('✅ LangChain OK')"
```

Si falta alguna:

```powershell
# Instalar dependencias básicas
pip install opencv-python numpy cv_bridge

# Instalar dependencias IA
pip install langchain-openai langgraph python-dotenv
```

### 3. Configurar API Key de OpenAI (Opcional)

Si quieres usar análisis con IA, crea un archivo `.env` en `MultiAgent/`:

```env
OPENAI_API_KEY=tu_api_key_aqui
```

O configura la variable de entorno:

```powershell
$env:OPENAI_API_KEY="tu_api_key_aqui"
```

---

## 🚀 Ejecutar el Sistema

Necesitas **3 terminales** abiertas simultáneamente:

### Terminal 1: Simulador MicroSim

```powershell
# Activar entorno
& "$env:LOCALAPPDATA\miniforge3\condabin\conda.bat" activate ros2_humble

# Ir al directorio
cd "D:\Proyectos PFG\robotic-ai-agents\simulator\microsim"

# Cargar workspace
& install\setup.bat

# Ejecutar simulador
ros2 run microsim microsim_node
```

**Qué esperar:**
- Deberías ver mensajes del simulador iniciando
- Carga del escenario
- El drone aparecerá en el mundo

### Terminal 2: Visualización (Opcional pero Recomendado)

```powershell
# Activar entorno
& "$env:LOCALAPPDATA\miniforge3\condabin\conda.bat" activate ros2_humble

# Ir al directorio
cd "D:\Proyectos PFG\robotic-ai-agents\simulator\microsim"

# Ejecutar visualización
python scripts\viz_2d.py
```

**Qué esperar:**
- Se abrirá una ventana 3D con visualización
- Verás el drone y el mundo desde arriba
- Puedes rotar la cámara con el mouse

### Terminal 3: Controlador UAV Inteligente

```powershell
# Activar entorno
& "$env:LOCALAPPDATA\miniforge3\condabin\conda.bat" activate ros2_humble

# Ir al directorio
cd "D:\Proyectos PFG\robotic-ai-agents\simulator\microsim"

# Cargar workspace
& install\setup.bat

# Ejecutar controlador
python scripts\uav_ai_controller.py
```

**Qué esperar:**
- Mensajes de inicialización del controlador
- Si tienes IA configurada: mensajes de análisis cada 2 segundos
- El drone comenzará a moverse siguiendo waypoints
- Si hay víctimas detectadas, verás mensajes de planificación

---

## 🔍 Qué Observar

### Sin IA (Solo Controlador Básico)

El controlador funcionará con waypoints predefinidos:
- El drone seguirá una ruta circular básica
- No habrá análisis de imágenes
- Los waypoints son los del controlador básico

### Con IA Configurada

Si tienes `OPENAI_API_KEY` configurada:
- **Cada 2 segundos:** Análisis de la imagen de la cámara
- **Detección:** Identificación de víctimas y obstáculos
- **Planificación:** Generación automática de waypoints
- **Comunicación:** Publicación de misión al UGV (si está escuchando)

### Salida Esperada (Con IA)

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

---

## 🐛 Troubleshooting

### Error: "No module named 'multiagent'"

**Solución:**
```powershell
# Asegúrate de estar en el directorio correcto
cd "D:\Proyectos PFG\robotic-ai-agents\simulator\microsim"

# Verifica que existe el directorio
dir multiagent
```

### Error: "llm is None"

**Causa:** No hay API key de OpenAI configurada.

**Solución:** 
- El sistema seguirá funcionando pero sin análisis IA
- O configura `OPENAI_API_KEY` como se indicó arriba

### Error: "Package 'microsim' not found"

**Solución:**
```powershell
# Asegúrate de cargar el workspace
& install\setup.bat

# Verifica que el paquete esté compilado
colcon build --packages-select microsim
```

### Error: "ros2 command not found"

**Solución:**
```powershell
# Activa el entorno conda
& "$env:LOCALAPPDATA\miniforge3\condabin\conda.bat" activate ros2_humble

# Carga ROS 2
call %CONDA_PREFIX%\etc\conda\activate.d\setup_ros2.bat
```

### El drone no se mueve

**Posibles causas:**
1. La misión no se ha iniciado automáticamente
2. No hay waypoints disponibles
3. El simulador no está corriendo

**Solución:**
- Verifica que el simulador esté corriendo (Terminal 1)
- Espera unos segundos después de iniciar el controlador
- Revisa los logs del controlador

---

## ✅ Verificación de Funcionamiento

### Checklist de Éxito

- [ ] Simulador iniciado sin errores
- [ ] Visualización muestra el mundo y el drone
- [ ] Controlador se conecta al simulador
- [ ] El drone comienza a moverse
- [ ] (Con IA) Se ven mensajes de análisis periódicos
- [ ] (Con IA) Se detectan víctimas/obstáculos
- [ ] (Con IA) Se genera y sigue una ruta planificada

### Monitorear Topics ROS 2

En una terminal adicional, puedes verificar los topics:

```powershell
# Ver todos los topics
ros2 topic list

# Ver mensajes de comando de velocidad
ros2 topic echo /drone/cmd_vel

# Ver misión publicada al UGV
ros2 topic echo /uav/mission_brief

# Ver imágenes de la cámara (cuidado, mucho output)
ros2 topic echo /drone/camera/image_raw --once
```

---

## 📊 Próximos Pasos

Una vez que funcione:

1. **Mejorar detección:** Ajustar prompts de GPT para mejor precisión
2. **Agregar UGV:** Implementar controlador UGV que escuche misiones
3. **Visualización mejorada:** Mostrar decisiones IA en la visualización
4. **Testing:** Crear escenarios de prueba automatizados

---

**¿Problemas?** Revisa los logs en cada terminal y compara con los mensajes esperados.

