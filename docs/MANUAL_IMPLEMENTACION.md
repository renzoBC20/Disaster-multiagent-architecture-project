# Manual de Implementación para Robots Reales
## Disaster Multi-Agent Architecture Project

**Versión:** 1.0  
**Fecha:** 2025  
**Autor:** Renzo BC20

---

## Tabla de Contenidos

1. [Introducción](#introducción)
2. [Especificaciones de Hardware - UAV](#especificaciones-de-hardware---uav)
3. [Especificaciones de Hardware - UGV](#especificaciones-de-hardware---ugv)
4. [Especificaciones de Software](#especificaciones-de-software)
5. [Sistema de Comunicación](#sistema-de-comunicación)
6. [Instalación y Configuración](#instalación-y-configuración)
7. [Consideraciones de Seguridad](#consideraciones-de-seguridad)
8. [Testing y Validación](#testing-y-validación)
9. [Mantenimiento y Operación](#mantenimiento-y-operación)
10. [Troubleshooting](#troubleshooting)

---

## 1. Introducción

Este manual proporciona las especificaciones técnicas detalladas necesarias para implementar la arquitectura multi-agente de rescate en robots reales. El sistema está diseñado para operar en entornos de desastre, requiriendo alta confiabilidad, redundancia y robustez.

### 1.1 Objetivo del Manual

Este documento especifica:
- Requisitos de hardware para UAV y UGV
- Stack de software y dependencias
- Configuración de sistemas de comunicación
- Procedimientos de instalación y despliegue
- Protocolos de seguridad y seguridad operativa
- Métodos de validación y testing

### 1.2 Alcance

El manual cubre la implementación física de:
- **UAV (Unmanned Aerial Vehicle)**: Vehículo aéreo no tripulado para reconocimiento aéreo
- **UGV (Unmanned Ground Vehicle)**: Vehículo terrestre no tripulado para rescate terrestre
- **Sistema de comunicación inter-agente**
- **Infraestructura de computación embebida**

---

## 2. Especificaciones de Hardware - UAV

### 2.1 Plataforma de Vuelo

#### 2.1.1 Tipo de Plataforma
- **Recomendado**: Multirotor (Quadcopter/Hexacopter)
- **Justificación**: Capacidad de vuelo estacionario, despegue y aterrizaje vertical, maniobrabilidad para reconocimiento aéreo

#### 2.1.2 Especificaciones Físicas

| Parámetro | Especificación Mínima | Especificación Recomendada |
|-----------|----------------------|---------------------------|
| **Peso Máximo al Despegue (MTOW)** | 2.5 kg | 3.5 - 5.0 kg |
| **Capacidad de Carga Útil** | 0.8 kg | 1.5 - 2.5 kg |
| **Dimensiones (Diagonal del Frame)** | 400 mm | 550 - 650 mm |
| **Tiempo de Vuelo** | 15 minutos | 25 - 35 minutos |
| **Autonomía Operativa** | 500 m | 1 - 2 km (línea de vista) |
| **Techo Operativo** | 50 m AGL | 120 m AGL |

#### 2.1.3 Propulsores y Motores

| Componente | Especificación |
|-----------|----------------|
| **Tipo de Motor** | Brushless DC (BLDC) sin escobillas |
| **Número de Motores** | 4 (Quadcopter) o 6 (Hexacopter) |
| **Potencia por Motor** | 200-300 W |
| **KV Rating** | 700-1000 KV |
| **Propulsores** | 10"-13" diámetro, paso 4.5"-5.5" |
| **Velocidad Máxima** | 15 m/s horizontal, 5 m/s vertical |

#### 2.1.4 Sistema de Batería

| Parámetro | Especificación |
|-----------|----------------|
| **Tipo** | LiPo (Lithium Polymer) o Li-Ion |
| **Configuración** | 4S (14.8V) o 6S (22.2V) |
| **Capacidad** | 5000-10000 mAh |
| **C-Rating** | Mínimo 25C (descarga continua), 50C burst |
| **Tiempo de Carga** | 60-90 minutos (cargador balanceado) |
| **Baterías de Repuesto** | Mínimo 3 baterías por misión |

### 2.2 Sistema de Computación Embebida

#### 2.2.1 Computadora de Vuelo Principal

| Componente | Especificación Mínima | Especificación Recomendada |
|-----------|----------------------|---------------------------|
| **Plataforma** | Raspberry Pi 4B (4GB RAM) | NVIDIA Jetson Nano/NX o Raspberry Pi 4B (8GB) |
| **CPU** | Quad-core ARM Cortex-A72 @ 1.5GHz | 6-core ARM Cortex-A78AE @ 1.4GHz (Jetson) |
| **RAM** | 4 GB LPDDR4 | 8 GB LPDDR4 |
| **GPU** | VideoCore VI | NVIDIA GPU (128 CUDA cores) |
| **Almacenamiento** | 32 GB microSD (Clase 10+) | 64-128 GB eMMC o NVMe SSD |
| **Sistema Operativo** | Ubuntu 22.04 LTS o ROS 2 Humble | Ubuntu 22.04 LTS + ROS 2 Humble |

#### 2.2.2 Controlador de Vuelo (Flight Controller)

| Parámetro | Especificación |
|-----------|----------------|
| **Plataforma** | Pixhawk 4, Pixhawk 6C, o equivalente |
| **Firmware** | ArduPilot o PX4 |
| **Procesador** | STM32F7 (168 MHz) o equivalente |
| **IMU** | 3-axis Gyroscope + 3-axis Accelerometer + Magnetómetro |
| **Barómetro** | Sensibilidad ±0.1 mbar |
| **GPS/Compass** | GPS + GLONASS, precisión <2 m |

#### 2.2.3 Módulos de Comunicación

**Wi-Fi 5G/4G LTE:**
- Módulo Wi-Fi 802.11ac (dual-band 2.4/5 GHz)
- Módem 4G LTE (opcional para comunicación a larga distancia)
- Antenas omnidireccionales de alta ganancia (5-8 dBi)

**Radio Telemetría:**
- Transceptor 915 MHz o 2.4 GHz (regulaciones locales)
- Alcance: 1-2 km (línea de vista)
- Tasa de datos: 57.6 kbps - 250 kbps

### 2.3 Sensores

#### 2.3.1 Sistema de Cámara

| Parámetro | Especificación Mínima | Especificación Recomendada |
|-----------|----------------------|---------------------------|
| **Tipo** | Cámara RGB digital con estabilización |
| **Resolución** | 1920x1080 @ 30 fps | 3840x2160 @ 30 fps o 1920x1080 @ 60 fps |
| **Sensor** | 1/2.3" CMOS | 1" CMOS o mayor |
| **Lente** | FOV 60-90° | FOV 84° ajustable |
| **Estabilización** | Gimbal 2-3 ejes | Gimbal 3 ejes con control PID |
| **Orientación** | Nadir (hacia abajo) | Nadir con capacidad de inclinación ±30° |
| **Formato de Salida** | H.264/H.265 | H.264/H.265 hardware-accelerated |
| **Interfaz** | USB 3.0 o MIPI CSI-2 | USB 3.0, MIPI CSI-2, o Ethernet |

**Ejemplos de Cámaras Recomendadas:**
- Raspberry Pi Camera Module 3 (resolución media, bajo costo)
- FLIR Blackfly S USB3 (alta resolución, industrial)
- Intel RealSense D435i (RGB + profundidad)

#### 2.3.2 GPS/GNSS

| Parámetro | Especificación |
|-----------|----------------|
| **Tipo** | GPS + GLONASS + Galileo (multi-constelación) |
| **Precisión Horizontal** | < 2 m (GPS standalone), < 1 m (RTK opcional) |
| **Precisión Vertical** | < 5 m |
| **Tasa de Actualización** | 10 Hz |
| **Tiempo de Adquisición** | < 30 segundos (frío), < 5 segundos (caliente) |
| **Compás Magnético** | 3-axis, precisión ±2° |

#### 2.3.3 Sensor de Altitud/Barómetro

| Parámetro | Especificación |
|-----------|----------------|
| **Tipo** | Barómetro MEMS |
| **Rango** | 300-1100 mbar |
| **Precisión** | ±0.1 mbar |
| **Tasa de Muestreo** | 100 Hz |

#### 2.3.4 Sensor Inercial (IMU)

| Parámetro | Especificación |
|-----------|----------------|
| **Tipo** | 9-DOF (Giroscopio 3-axis + Acelerómetro 3-axis + Magnetómetro 3-axis) |
| **Giroscopio** | Rango ±2000°/s, precisión ±0.1°/s |
| **Acelerómetro** | Rango ±16g, precisión ±0.01g |
| **Tasa de Muestreo** | 400-1000 Hz |
| **Filtro de Fusión** | Sensor fusion (complementary/Kalman filter) |

### 2.4 Sistemas de Seguridad

#### 2.4.1 Failsafe y Recuperación
- **Return-to-Home (RTH)**: Activación automática en pérdida de señal
- **Geofencing**: Límites de área operativa programables
- **Batería Baja**: Aterrizaje automático cuando batería < 20%
- **Paracaídas de Recuperación**: Opcional para operaciones sobre áreas pobladas

#### 2.4.2 Indicadores Visuales y Sonoros
- LEDs de estado (posición, batería, GPS lock)
- Buzzer para localización acústica
- Beacon de localización (opcional)

---

## 3. Especificaciones de Hardware - UGV

### 3.1 Plataforma Terrestre

#### 3.1.1 Tipo de Plataforma

**Recomendado**: Robot terrestre con tracción diferencial o orugas  
**Justificación**: Capacidad off-road, estabilidad, capacidad de carga

#### 3.1.2 Especificaciones Físicas

| Parámetro | Especificación Mínima | Especificación Recomendada |
|-----------|----------------------|---------------------------|
| **Peso Total** | 15 kg | 25-40 kg |
| **Capacidad de Carga Útil** | 5 kg | 10-15 kg |
| **Dimensiones (L×W×H)** | 60×50×30 cm | 80×60×40 cm |
| **Velocidad Máxima** | 1.0 m/s | 1.5-2.0 m/s |
| **Velocidad de Crucero** | 0.5 m/s | 1.0 m/s |
| **Gradiente Máximo** | 15° | 30° |
| **Autonomía Operativa** | 2 horas | 4-6 horas |
| **Rango Operativo** | 500 m | 1-2 km (línea de vista) |

#### 3.1.3 Sistema de Propulsión

| Componente | Especificación |
|-----------|----------------|
| **Tipo de Tracción** | Diferencial (2 ruedas motrices) o orugas |
| **Motores** | 2× BLDC de 200-500 W cada uno |
| **Reductor** | Gearbox con relación 10:1 a 20:1 |
| **Controlador de Motor** | ESC (Electronic Speed Controller) dual, 30-50 A |
| **Encoders** | Encoders incrementales en ruedas (1000 PPR) |
| **Suspensión** | Sistema de suspensión independiente o orugas articuladas |

#### 3.1.4 Sistema de Batería

| Parámetro | Especificación |
|-----------|----------------|
| **Tipo** | LiPo o LiFePO4 (mayor seguridad) |
| **Configuración** | 4S (14.8V) o 6S (22.2V) |
| **Capacidad** | 20,000-30,000 mAh |
| **C-Rating** | Mínimo 20C (descarga continua) |
| **Tiempo de Carga** | 2-3 horas (cargador balanceado) |
| **Baterías de Repuesto** | Mínimo 2 baterías por misión |

### 3.2 Sistema de Computación Embebida

#### 3.2.1 Computadora Principal

| Componente | Especificación Mínima | Especificación Recomendada |
|-----------|----------------------|---------------------------|
| **Plataforma** | Raspberry Pi 4B (4GB RAM) | NVIDIA Jetson Nano/NX o Raspberry Pi 4B (8GB) |
| **CPU** | Quad-core ARM Cortex-A72 @ 1.5GHz | 6-core ARM Cortex-A78AE @ 1.4GHz (Jetson) |
| **RAM** | 4 GB LPDDR4 | 8 GB LPDDR4 |
| **GPU** | VideoCore VI | NVIDIA GPU (128 CUDA cores) |
| **Almacenamiento** | 64 GB microSD (Clase 10+) o SSD | 128-256 GB eMMC o NVMe SSD |
| **Sistema Operativo** | Ubuntu 22.04 LTS + ROS 2 Humble | Ubuntu 22.04 LTS + ROS 2 Humble |

#### 3.2.2 Microcontrolador Auxiliar

**Arduino o STM32:**
- Control de bajo nivel de motores y sensores
- Interfaz con encoders y ESCs
- Gestión de energía y monitoreo de batería
- Comunicación serial con computadora principal (UART/USB)

### 3.3 Sensores

#### 3.3.1 Sistema de Cámara

| Parámetro | Especificación Mínima | Especificación Recomendada |
|-----------|----------------------|---------------------------|
| **Tipo** | Cámara RGB digital con estabilización |
| **Resolución** | 1920x1080 @ 30 fps | 1920x1080 @ 60 fps |
| **Sensor** | 1/2.3" CMOS | 1" CMOS |
| **Lente** | FOV 60-90° | FOV 84-120° (gran angular) |
| **Orientación** | Frontal con inclinación ajustable | Frontal + cámara auxiliar trasera |
| **Estabilización** | Digital (software) | Gimbal 2 ejes o estabilización mecánica |
| **Interfaz** | USB 3.0 o MIPI CSI-2 | USB 3.0 o Ethernet |

#### 3.3.2 Sensor de Rango (LiDAR/Ultrasonido)

**LiDAR 2D (Recomendado):**
| Parámetro | Especificación |
|-----------|----------------|
| **Tipo** | LiDAR 2D rotatorio |
| **Rango** | 8-12 m |
| **Resolución Angular** | 0.25-0.5° |
| **Tasa de Escaneo** | 10-20 Hz |
| **FOV Horizontal** | 360° |
| **Ejemplos** | RPLIDAR A1, YDLIDAR X4, Hokuyo URG-04LX |

**Sensor Ultrasónico (Alternativa/Economía):**
- Rango: 2-4 m
- 4-8 sensores distribuidos alrededor del robot
- Tasa de actualización: 20-40 Hz

#### 3.3.3 GPS/GNSS

| Parámetro | Especificación |
|-----------|----------------|
| **Tipo** | GPS + GLONASS + Galileo (multi-constelación) |
| **Precisión Horizontal** | < 2 m (GPS standalone), < 0.5 m (RTK opcional) |
| **Precisión Vertical** | < 5 m |
| **Tasa de Actualización** | 10 Hz |
| **Compás Magnético** | 3-axis, precisión ±2° |

#### 3.3.4 Sensor Inercial (IMU)

| Parámetro | Especificación |
|-----------|----------------|
| **Tipo** | 9-DOF (Giroscopio + Acelerómetro + Magnetómetro) |
| **Giroscopio** | Rango ±2000°/s, precisión ±0.1°/s |
| **Acelerómetro** | Rango ±16g, precisión ±0.01g |
| **Tasa de Muestreo** | 100-400 Hz |
| **Filtro de Fusión** | Sensor fusion (Kalman filter) |

#### 3.3.5 Sensores Adicionales

**Odometría Visual (Opcional):**
- Cámara estéreo o monocámara con odometría visual (VO)
- Para SLAM y navegación en interiores/GPS-denied

**Sensor de Inclinación:**
- Inclinómetro para detectar gradientes y prevenir vuelcos
- Precisión: ±1°

### 3.4 Sistemas de Seguridad

#### 3.4.1 Sistema de Parada de Emergencia (E-Stop)
- Botón físico de parada de emergencia
- Interruptor de seguridad inalámbrico (opcional)
- Parada automática en pérdida de comunicación

#### 3.4.2 Protección Mecánica
- Parachoques delanteros y laterales
- Protección contra sobrecarga (fuses/circuit breakers)
- Sistema de frenado de emergencia

#### 3.4.3 Indicadores
- LEDs de estado (batería, GPS, comunicación)
- Pantalla LCD opcional para monitoreo local
- Alarmas sonoras para advertencias

---

## 4. Especificaciones de Software

### 4.1 Sistema Operativo

| Plataforma | Versión | Notas |
|-----------|---------|-------|
| **Ubuntu Linux** | 22.04 LTS (Jammy Jellyfish) | Recomendado para desarrollo y producción |
| **ROS 2** | Humble Hawksbill | Framework principal de robótica |

### 4.2 Stack de Software Principal

#### 4.2.1 Sistema Base ROS 2

```yaml
ros-humble-desktop: Full desktop installation
ros-humble-cv-bridge: OpenCV bridge para imágenes
ros-humble-image-transport: Transporte eficiente de imágenes
ros-humble-geometry-msgs: Mensajes de geometría
ros-humble-nav-msgs: Mensajes de navegación
ros-humble-sensor-msgs: Mensajes de sensores
ros-humble-tf2: Transformaciones de coordenadas
ros-humble-tf2-ros: ROS 2 wrapper para TF2
```

#### 4.2.2 Control de Vuelo (UAV)

**ArduPilot o PX4:**
- Firmware de control de vuelo autónomo
- Integración con ROS 2 mediante MAVROS (ArduPilot) o PX4 ROS 2 Interface

**MAVROS (para ArduPilot):**
```bash
ros-humble-mavros
ros-humble-mavros-extras
```

**PX4 ROS 2 Interface:**
```bash
px4_ros_com: PX4 ROS 2 communication package
```

#### 4.2.3 Procesamiento de Imágenes

**OpenCV:**
```bash
libopencv-dev (v4.8.0+)
python3-opencv
```

**Librerías Adicionales:**
```bash
cv_bridge: Conversión ROS Image ↔ OpenCV
image_transport: Compresión y transporte de imágenes
```

#### 4.2.4 Inteligencia Artificial y Machine Learning

**LangGraph y LangChain:**
```bash
langgraph>=0.6.0
langchain-openai>=0.3.0
langchain-core>=0.3.0
```

**OpenAI API Client:**
```bash
openai>=1.0.0
httpx>=0.24.0
```

#### 4.2.5 Utilidades Python

```bash
numpy>=1.24.0
python-dotenv>=1.0.0
pydantic>=2.0.0
pillow>=10.0.0
orjson>=3.9.0
anyio>=4.0.0
requests>=2.31.0
```

### 4.3 Dependencias del Sistema

#### 4.3.1 Librerías del Sistema (Ubuntu/Debian)

```bash
# Build tools
build-essential
cmake
pkg-config

# Python development
python3-dev
python3-pip
python3-venv

# Multimedia
libavcodec-dev
libavformat-dev
libavutil-dev
libswscale-dev

# Networking
libasio-dev
libtinyxml2-dev

# Serial communication
libserial-dev
```

#### 4.3.2 Gestión de Entornos Virtuales

**Python Virtual Environment:**
```bash
python3 -m venv venv
source venv/bin/activate  # Linux/Mac
venv\Scripts\activate     # Windows
```

### 4.4 Configuración de Red

#### 4.4.1 Requisitos de Conectividad

**UAV:**
- Conexión Wi-Fi 802.11ac (dual-band)
- Opcional: Módem 4G LTE para operaciones a larga distancia
- Radio telemetría 915 MHz/2.4 GHz

**UGV:**
- Conexión Wi-Fi 802.11ac
- Opcional: Módem 4G LTE
- Radio telemetría 915 MHz/2.4 GHz

**Estación Base:**
- Conexión Ethernet o Wi-Fi estable
- Acceso a Internet para API de OpenAI
- Router/modem para comunicación con robots

#### 4.4.2 Configuración de Red ROS 2

**Configurar variables de entorno:**
```bash
export ROS_DOMAIN_ID=0  # Usar mismo dominio para todos los nodos
export ROS_DISCOVERY_SERVER=  # Opcional: servidor de descubrimiento centralizado
```

**Configurar IP estática (recomendado):**
```bash
# UAV
IP: 192.168.1.100
Netmask: 255.255.255.0
Gateway: 192.168.1.1

# UGV
IP: 192.168.1.101
Netmask: 255.255.255.0
Gateway: 192.168.1.1

# Estación Base
IP: 192.168.1.1
```

### 4.5 Gestión de Configuración

#### 4.5.1 Variables de Entorno

**Archivo `.env` (en cada robot):**
```bash
# OpenAI API
OPENAI_API_KEY=sk-...

# ROS 2 Configuration
ROS_DOMAIN_ID=0

# Robot Identification
ROBOT_TYPE=uav  # o ugv
ROBOT_ID=uav_001

# Network Configuration
ROBOT_IP=192.168.1.100
BASE_STATION_IP=192.168.1.1

# Sensor Configuration
CAMERA_RESOLUTION_WIDTH=256
CAMERA_RESOLUTION_HEIGHT=256
CAMERA_FPS=10
GPS_RATE_HZ=10
```

#### 4.5.2 Archivos de Configuración YAML

**Escenario de misión (`mission_scenario.yaml`):**
```yaml
world:
  size: [100.0, 100.0]  # metros

robots:
  drone:
    initial_pose:
      x: 0.0
      y: 0.0
      z: 5.0
      yaw: 0.0
    max_velocity: 3.0
    max_accel: 2.0

  rover:
    initial_pose:
      x: 0.0
      y: 0.0
      theta: 0.0
    max_velocity: 1.0
    max_omega: 1.57

sensors:
  gps:
    rate_hz: 10.0
    noise_std: 0.1
  
  camera:
    width: 256
    height: 256
    fov_deg: 60.0
    rate_hz: 10.0

radio:
  max_range: 1000.0  # metros
  base_latency: 0.05  # segundos
  jitter: 0.01
  packet_loss: 0.01
```

---

## 5. Sistema de Comunicación

### 5.1 Arquitectura de Comunicación

```
┌─────────────────┐         ┌─────────────────┐
│   Estación      │         │   Estación      │
│      Base       │◄───────►│      Base       │
│  (Monitoreo)    │   Wi-Fi │  (Control)      │
└─────────────────┘         └─────────────────┘
         │                           │
         │                           │
    Wi-Fi/4G                   Radio/4G
         │                           │
         ▼                           ▼
┌─────────────────┐         ┌─────────────────┐
│      UAV        │◄───────►│      UGV        │
│                 │  Radio  │                 │
│                 │ Telemetría                │
└─────────────────┘         └─────────────────┘
```

### 5.2 Protocolos de Comunicación

#### 5.2.1 ROS 2 Topics (Comunicación Inter-Agente)

**Topics del UAV:**
```bash
/drone/cmd_vel              # Comandos de velocidad (Twist)
/drone/odom                 # Odometría
/drone/gps/fix              # Posición GPS
/drone/camera/image_raw     # Imagen de cámara
/drone/camera/camera_info   # Información de cámara
/uav/mission_brief          # Briefing de misión (String)
```

**Topics del UGV:**
```bash
/rover/cmd_vel              # Comandos de velocidad
/rover/odom                 # Odometría
/rover/gps/fix              # Posición GPS
/rover/range                # Sensor de rango
/rover/camera/image_raw     # Imagen de cámara
```

**Topics de Comunicación Inter-Agente:**
```bash
/radio/uav_tx               # Mensajes del UAV
/radio/uav_rx               # Mensajes recibidos por UAV
/radio/ugv_tx               # Mensajes del UGV
/radio/ugv_rx               # Mensajes recibidos por UGV
```

#### 5.2.2 Formato de Mensajes

**Mensaje de Misión (JSON):**
```json
{
  "timestamp": "2025-01-15 14:30:00",
  "from": "UAV",
  "to": "UGV",
  "message_type": "MISSION_BRIEFING",
  "mission_brief": "...",
  "status": "READY_FOR_UGV",
  "message_id": "MSG_1234567890"
}
```

#### 5.2.3 Radio Telemetría

**Especificaciones:**
- **Frecuencia**: 915 MHz (América) o 868 MHz (Europa) o 2.4 GHz
- **Protocolo**: LoRa, SiK Radio, o XBee
- **Alcance**: 1-2 km (línea de vista)
- **Tasa de Datos**: 57.6 kbps - 250 kbps
- **Modo**: Half-duplex

**Hardware Recomendado:**
- **3DR Radio Telemetry** (SiK Radio)
- **RFD900x** (LoRa, largo alcance)
- **XBee Pro** (Zigbee, corto alcance)

### 5.3 QoS y Confiabilidad

**Configuración QoS ROS 2:**
```python
# Comandos críticos (reliable)
qos_profile = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=10
)

# Datos de sensores (best effort para reducir latencia)
qos_sensor = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=5
)
```

---

## 6. Instalación y Configuración

### 6.1 Preparación del Sistema Operativo

#### 6.1.1 Instalación de Ubuntu 22.04 LTS

1. **Descargar imagen ISO** desde ubuntu.com
2. **Crear USB booteable** (usando balenaEtcher o similar)
3. **Instalar en disco SSD** (no microSD para producción)
4. **Configurar particiones:**
   - `/` (root): 20-30 GB
   - `/home`: Resto del espacio
   - Swap: 4-8 GB

#### 6.1.2 Configuración Inicial

```bash
# Actualizar sistema
sudo apt update && sudo apt upgrade -y

# Instalar herramientas básicas
sudo apt install -y git curl wget vim build-essential

# Configurar hostname
sudo hostnamectl set-hostname uav-001  # o ugv-001
```

### 6.2 Instalación de ROS 2 Humble

#### 6.2.1 Instalación Base

```bash
# Configurar locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Agregar repositorio ROS 2
sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Agregar clave GPG
sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Agregar repositorio
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'

# Instalar ROS 2
sudo apt update
sudo apt install -y ros-humble-desktop

# Configurar entorno
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### 6.2.2 Dependencias Adicionales

```bash
# Instalar herramientas de desarrollo ROS 2
sudo apt install -y python3-colcon-common-extensions \
    python3-rosdep python3-vcstool

# Inicializar rosdep
sudo rosdep init
rosdep update

# Instalar dependencias de comunicación
sudo apt install -y ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-tf2 \
    ros-humble-tf2-ros
```

### 6.3 Instalación del Proyecto

#### 6.3.1 Clonar Repositorio

```bash
# Crear workspace
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# Clonar proyecto
git clone https://github.com/renzoBC20/Disaster-multiagent-architecture-project.git

# O solo el simulador
git clone <repository-url> robotic-ai-agents
```

#### 6.3.2 Instalar Dependencias Python

```bash
# Crear entorno virtual
cd ~/robot_ws/src/Disaster-multiagent-architecture-project/MultiAgent
python3 -m venv venv
source venv/bin/activate

# Instalar dependencias
pip install --upgrade pip
pip install -r requirements.txt
```

#### 6.3.3 Compilar Paquete ROS 2

```bash
# Regresar a workspace
cd ~/robot_ws

# Instalar dependencias del sistema
rosdep install --from-paths src --ignore-src -r -y

# Compilar
colcon build --packages-select microsim

# Configurar entorno
source install/setup.bash
echo "source ~/robot_ws/install/setup.bash" >> ~/.bashrc
```

### 6.4 Configuración de Hardware Específico

#### 6.4.1 UAV - Configuración de Flight Controller

**ArduPilot:**
1. Conectar Pixhawk vía USB
2. Abrir Mission Planner o QGroundControl
3. Configurar parámetros:
   - Frame type: Quadcopter X
   - Motor outputs: Q_MOTOR_COUNT = 4
   - Radio calibration
   - Accelerometer calibration
   - Compass calibration
   - GPS setup

**Instalar MAVROS:**
```bash
sudo apt install -y ros-humble-mavros ros-humble-mavros-extras
```

**Configurar conexión serial:**
```bash
# Agregar usuario a grupo dialout
sudo usermod -a -G dialout $USER

# Configurar udev rules para Pixhawk
sudo bash -c 'cat > /etc/udev/rules.d/99-pixhawk.rules << EOF
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0011", MODE="0666", GROUP="dialout"
EOF'
sudo udevadm control --reload-rules
```

#### 6.4.2 UGV - Configuración de Controladores de Motor

**Configurar comunicación serial:**
```bash
# Identificar puerto USB del controlador
lsusb
dmesg | grep tty

# Configurar permisos
sudo chmod 666 /dev/ttyUSB0  # o ttyACM0
```

**Configurar Arduino/STM32:**
- Cargar firmware de control de motores
- Configurar parámetros (velocidades máximas, PID, etc.)
- Probar comunicación serial

#### 6.4.3 Configuración de Cámara

**Raspberry Pi Camera:**
```bash
# Habilitar cámara
sudo raspi-config
# Interface Options → Camera → Enable

# Probar cámara
raspistill -o test.jpg
```

**Cámara USB:**
```bash
# Verificar detección
lsusb | grep -i camera
v4l2-ctl --list-devices

# Probar captura
v4l2-ctl --device=/dev/video0 --stream-mmap --stream-count=1 --stream-to=test.raw
```

#### 6.4.4 Configuración de GPS

```bash
# Verificar puerto GPS
sudo apt install -y gpsd gpsd-clients
sudo systemctl stop gpsd.socket
sudo systemctl disable gpsd.socket

# Configurar gpsd
sudo gpsd /dev/ttyUSB0 -F /var/run/gpsd.sock

# Probar GPS
cgps -s
```

### 6.5 Configuración de Red

#### 6.5.1 Configurar Wi-Fi

```bash
# Editar configuración de red
sudo nano /etc/netplan/50-cloud-init.yaml

# Agregar configuración:
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: false
      addresses:
        - 192.168.1.100/24  # Cambiar según robot
      gateway4: 192.168.1.1
      nameservers:
        addresses:
          - 8.8.8.8
          - 8.8.4.4
      access-points:
        "SSID_NAME":
          password: "PASSWORD"

# Aplicar configuración
sudo netplan apply
```

#### 6.5.2 Configurar Radio Telemetría

**3DR Radio (SiK):**
```bash
# Instalar herramientas
sudo apt install -y screen

# Conectar vía USB
screen /dev/ttyUSB0 57600

# En modo AT, configurar:
# AT&S1=1  # Guardar configuración
# AT&W     # Escribir a EEPROM
```

### 6.6 Configuración de API Keys

```bash
# Crear archivo de configuración
cd ~/robot_ws/src/Disaster-multiagent-architecture-project/MultiAgent
cp config.example .env

# Editar con API key
nano .env
# Agregar: OPENAI_API_KEY=sk-...

# Asegurar permisos
chmod 600 .env
```

### 6.7 Configuración de Servicios del Sistema

#### 6.7.1 Crear Servicio systemd para Nodos ROS 2

**Archivo: `/etc/systemd/system/uav-controller.service`**
```ini
[Unit]
Description=UAV LangGraph Controller
After=network.target

[Service]
Type=simple
User=robot
WorkingDirectory=/home/robot/robot_ws
Environment="ROS_DOMAIN_ID=0"
Environment="PATH=/home/robot/robot_ws/src/Disaster-multiagent-architecture-project/MultiAgent/venv/bin:/usr/bin:/bin"
ExecStart=/home/robot/robot_ws/src/Disaster-multiagent-architecture-project/MultiAgent/venv/bin/python3 /home/robot/robot_ws/src/Disaster-multiagent-architecture-project/robotic-ai-agents/simulator/microsim/scripts/uav_langgraph_controller.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

**Habilitar servicio:**
```bash
sudo systemctl daemon-reload
sudo systemctl enable uav-controller.service
sudo systemctl start uav-controller.service
```

---

## 7. Consideraciones de Seguridad

### 7.1 Seguridad Operativa

#### 7.1.1 Checklist Pre-Vuelo (UAV)

- [ ] Verificar nivel de batería (>80%)
- [ ] Inspección visual del frame y propulsores
- [ ] Calibración de IMU y compás
- [ ] Verificar GPS lock (>6 satélites)
- [ ] Prueba de motores (sin propulsores)
- [ ] Verificar comunicación con estación base
- [ ] Configurar geofencing
- [ ] Verificar failsafe (RTH)
- [ ] Revisar condiciones climáticas
- [ ] Verificar área de operación (sin obstáculos, sin personas)

#### 7.1.2 Checklist Pre-Operación (UGV)

- [ ] Verificar nivel de batería (>80%)
- [ ] Inspección visual del chasis y ruedas/orugas
- [ ] Calibración de sensores (IMU, GPS)
- [ ] Prueba de motores y dirección
- [ ] Verificar comunicación con estación base
- [ ] Prueba de sensor de rango/LiDAR
- [ ] Verificar E-Stop funcional
- [ ] Revisar área de operación

### 7.2 Seguridad de Software

#### 7.2.1 Protección de Credenciales

```bash
# Usar variables de entorno (no hardcodear)
export OPENAI_API_KEY=sk-...

# Usar archivos de configuración con permisos restringidos
chmod 600 .env

# No commitear archivos con credenciales
# Agregar a .gitignore:
.env
*.key
config
```

#### 7.2.2 Actualizaciones de Seguridad

```bash
# Actualizaciones automáticas de seguridad
sudo apt install -y unattended-upgrades
sudo dpkg-reconfigure -plow unattended-upgrades

# Configurar firewall
sudo ufw enable
sudo ufw allow 22/tcp  # SSH
sudo ufw allow from 192.168.1.0/24  # Red local
```

### 7.3 Seguridad de Comunicación

#### 7.3.1 Encriptación de Comunicación

- Usar **VPN** para comunicación Wi-Fi/Internet
- **WPA3** para redes Wi-Fi
- **HTTPS/TLS** para comunicación con APIs externas

#### 7.3.2 Autenticación

- Usar claves SSH en lugar de contraseñas
- Implementar autenticación mutua entre robots
- Rotar credenciales regularmente

### 7.4 Seguridad Física

- Almacenar robots en área segura
- Proteger contra acceso no autorizado
- Mantener registro de uso y mantenimiento
- Etiquetar baterías y seguir protocolos de seguridad LiPo

---

## 8. Testing y Validación

### 8.1 Testing de Hardware

#### 8.1.1 UAV - Pruebas en Tierra

```bash
# Prueba de sensores
ros2 run microsim test_sensors

# Prueba de comunicación
ros2 topic echo /drone/gps/fix
ros2 topic echo /drone/camera/image_raw

# Prueba de control
ros2 topic pub /drone/cmd_vel geometry_msgs/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

#### 8.1.2 UGV - Pruebas en Tierra

```bash
# Prueba de motores (sin carga)
ros2 topic pub /rover/cmd_vel geometry_msgs/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {z: 0.0}}"

# Prueba de sensores
ros2 topic echo /rover/range
ros2 topic echo /rover/gps/fix
```

### 8.2 Testing de Software

#### 8.2.1 Pruebas Unitarias

```bash
cd ~/robot_ws
colcon test --packages-select microsim
colcon test-result --verbose
```

#### 8.2.2 Pruebas de Integración

```bash
# Iniciar simulador (para pruebas)
ros2 run microsim microsim_node

# En otra terminal, ejecutar controlador
ros2 run microsim uav_langgraph_controller
```

### 8.3 Pruebas de Campo

#### 8.3.1 Fase 1: Pruebas Básicas

1. **Despegue y Aterrizaje Manual** (UAV)
2. **Movimiento Básico** (UGV)
3. **Comunicación Inter-Agente**
4. **Captura de Imágenes**

#### 8.3.2 Fase 2: Pruebas Autónomas

1. **Navegación Autónoma** (waypoints)
2. **Detección de Obstáculos**
3. **Comunicación de Misión**
4. **Ejecución de Ruta**

#### 8.3.3 Fase 3: Pruebas de Misión Completa

1. **Reconocimiento Aéreo Completo**
2. **Identificación de Víctimas/Obstáculos**
3. **Planificación de Ruta**
4. **Ejecución de Rescate**

### 8.4 Métricas de Validación

| Métrica | Objetivo | Método de Medición |
|---------|----------|-------------------|
| **Precisión de GPS** | < 2 m | Comparar con referencia RTK |
| **Tasa de Detección de Víctimas** | > 90% | Comparar con ground truth |
| **Tasa de Falsos Positivos** | < 5% | Análisis manual |
| **Tiempo de Comunicación** | < 500 ms | Timestamps de mensajes |
| **Confiabilidad del Sistema** | > 95% uptime | Logs del sistema |

---

## 9. Mantenimiento y Operación

### 9.1 Mantenimiento Preventivo

#### 9.1.1 UAV - Checklist Semanal

- [ ] Inspección visual del frame
- [ ] Limpieza de motores y propulsores
- [ ] Verificación de conexiones eléctricas
- [ ] Calibración de sensores
- [ ] Actualización de software
- [ ] Prueba de baterías

#### 9.1.2 UGV - Checklist Semanal

- [ ] Inspección visual del chasis
- [ ] Limpieza de ruedas/orugas
- [ ] Verificación de suspensión
- [ ] Calibración de sensores
- [ ] Verificación de E-Stop
- [ ] Prueba de baterías

### 9.2 Logs y Monitoreo

#### 9.2.1 Configuración de Logging

```bash
# Configurar ROS 2 logging
export RCUTILS_LOGGING_SEVERITY=INFO
export RCUTILS_COLORIZED_OUTPUT=1

# Redirigir logs a archivo
ros2 run microsim uav_langgraph_controller 2>&1 | tee ~/logs/uav_$(date +%Y%m%d_%H%M%S).log
```

#### 9.2.2 Monitoreo del Sistema

```bash
# Monitoreo de recursos
htop

# Monitoreo de temperatura (Jetson)
tegrastats

# Monitoreo de red
iftop
```

### 9.3 Actualizaciones

#### 9.3.1 Actualización de Software

```bash
# Actualizar sistema
sudo apt update && sudo apt upgrade -y

# Actualizar código del proyecto
cd ~/robot_ws/src/Disaster-multiagent-architecture-project
git pull origin main

# Recompilar
cd ~/robot_ws
colcon build --packages-select microsim
```

---

## 10. Troubleshooting

### 10.1 Problemas Comunes - UAV

#### Problema: No despega / Motores no responden

**Diagnóstico:**
```bash
# Verificar comunicación con flight controller
ros2 topic echo /mavros/state

# Verificar comandos
ros2 topic echo /drone/cmd_vel
```

**Soluciones:**
- Verificar calibración de radio control
- Revisar conexiones de ESCs a flight controller
- Verificar calibración de IMU
- Revisar modo de vuelo (debe estar en GUIDED/STABILIZE)

#### Problema: Pérdida de GPS

**Diagnóstico:**
```bash
ros2 topic echo /drone/gps/fix
```

**Soluciones:**
- Mover a área abierta (sin obstáculos)
- Esperar > 30 segundos para adquisición
- Verificar conexión de antena GPS
- Revisar configuración de GPS en flight controller

### 10.2 Problemas Comunes - UGV

#### Problema: Motores no responden

**Diagnóstico:**
```bash
# Verificar comunicación serial
dmesg | grep tty

# Verificar comandos
ros2 topic echo /rover/cmd_vel
```

**Soluciones:**
- Verificar conexiones de ESCs
- Revisar comunicación serial con controlador
- Verificar alimentación de motores
- Revisar configuración de permisos de puerto serial

#### Problema: Sensor de rango no funciona

**Diagnóstico:**
```bash
ros2 topic echo /rover/range
ros2 topic list | grep range
```

**Soluciones:**
- Verificar conexión de LiDAR/US
- Revisar alimentación del sensor
- Verificar configuración de puerto USB/serial
- Revisar drivers del sensor

### 10.3 Problemas de Comunicación

#### Problema: Robots no se comunican

**Diagnóstico:**
```bash
# Verificar red
ping 192.168.1.100  # IP del otro robot

# Verificar ROS 2 discovery
ros2 node list
```

**Soluciones:**
- Verificar mismo ROS_DOMAIN_ID
- Revisar configuración de red (IPs, máscara)
- Verificar firewall
- Revisar conexión Wi-Fi/radio

### 10.4 Problemas de Software

#### Problema: Nodos no inician

**Diagnóstico:**
```bash
# Verificar errores
journalctl -u uav-controller.service -n 50

# Verificar dependencias
rosdep check --from-paths src --ignore-src
```

**Soluciones:**
- Verificar instalación de dependencias
- Revisar configuración de servicios systemd
- Verificar permisos de archivos
- Revisar logs de errores

---

## Apéndices

### A. Glosario de Términos

- **AGL**: Above Ground Level (altura sobre el nivel del suelo)
- **BLDC**: Brushless DC Motor
- **ESC**: Electronic Speed Controller
- **FOV**: Field of View (campo de visión)
- **GNSS**: Global Navigation Satellite System
- **IMU**: Inertial Measurement Unit
- **LiPo**: Lithium Polymer (batería)
- **MTOW**: Maximum Take-Off Weight
- **RTK**: Real-Time Kinematic (GPS de alta precisión)
- **SLAM**: Simultaneous Localization and Mapping

### B. Referencias y Recursos

- **ROS 2 Documentation**: https://docs.ros.org/en/humble/
- **ArduPilot Documentation**: https://ardupilot.org/
- **PX4 Documentation**: https://docs.px4.io/
- **OpenCV Documentation**: https://docs.opencv.org/
- **LangGraph Documentation**: https://python.langchain.com/docs/langgraph

### C. Contacto y Soporte

- **Repositorio**: https://github.com/renzoBC20/Disaster-multiagent-architecture-project
- **Issues**: https://github.com/renzoBC20/Disaster-multiagent-architecture-project/issues
- **Autor**: Renzo BC20

---

**Versión del Documento:** 1.0  
**Última Actualización:** 2025  
**Estado:** Documento de Referencia

