# Manual de Implementación para Robots Reales
## Disaster Multi-Agent Architecture Project

**Versión:** 1.0  
**Fecha:** 2025  
**Autor:** Renzo BC20

---

## Tabla de Contenidos

1. [Introducción](#introducción)
2. [Requisitos del Sistema de Computación](#requisitos-del-sistema-de-computación)
3. [Requisitos de Sensores para la Arquitectura](#requisitos-de-sensores-para-la-arquitectura)
4. [Especificaciones de Software](#especificaciones-de-software)
5. [Sistema de Comunicación](#sistema-de-comunicación)
6. [Instalación y Configuración de la Arquitectura](#instalación-y-configuración-de-la-arquitectura)
7. [Configuración de Integración con Robots](#configuración-de-integración-con-robots)
8. [Testing y Validación](#testing-y-validación)
9. [Troubleshooting](#troubleshooting)
10. [Referencias](#referencias)

---

## 1. Introducción

Este manual proporciona las especificaciones técnicas necesarias para implementar la **arquitectura de software multi-agente** en robots reales. El documento se enfoca en los requisitos del sistema de computación, sensores necesarios para la arquitectura, y la configuración del stack de software, **no en las especificaciones físicas de los robots** (motores, propulsores, baterías, etc.).

### 1.1 Objetivo del Manual

Este documento especifica:
- Requisitos de hardware de computación embebida
- Sensores necesarios para ejecutar la arquitectura
- Stack de software completo y dependencias
- Configuración de sistemas de comunicación
- Procedimientos de instalación y despliegue del software
- Integración con robots existentes mediante ROS 2
- Métodos de validación y testing del software

### 1.2 Alcance

El manual cubre la implementación de la arquitectura de software en:
- **UAV (Unmanned Aerial Vehicle)**: Configuración del sistema de computación y sensores para reconocimiento aéreo
- **UGV (Unmanned Ground Vehicle)**: Configuración del sistema de computación y sensores para rescate terrestre
- **Sistema de comunicación inter-agente**: Protocolos y configuración
- **Infraestructura de software**: Instalación y configuración completa

**Nota:** Este manual asume que ya tienes un robot (UAV o UGV) funcional. Se enfoca únicamente en los componentes necesarios para ejecutar la arquitectura de software.

---

## 2. Requisitos del Sistema de Computación

### 2.1 Computadora Embebida Principal

La arquitectura requiere un sistema de computación embebida capaz de ejecutar ROS 2, procesar imágenes, y ejecutar workflows de LangGraph.

#### 2.1.1 Especificaciones Mínimas

| Componente | Especificación Mínima |
|-----------|----------------------|
| **Plataforma** | Raspberry Pi 4B (4GB RAM) |
| **CPU** | Quad-core ARM Cortex-A72 @ 1.5GHz |
| **RAM** | 4 GB LPDDR4 |
| **GPU** | VideoCore VI (procesamiento básico de imágenes) |
| **Almacenamiento** | 32 GB microSD (Clase 10+) o mejor SSD |
| **Conectividad** | Wi-Fi 802.11ac, Ethernet, USB 3.0 |
| **Sistema Operativo** | Ubuntu 22.04 LTS (ARM64) |

#### 2.1.2 Especificaciones Recomendadas

| Componente | Especificación Recomendada |
|-----------|---------------------------|
| **Plataforma** | NVIDIA Jetson Nano/NX o Raspberry Pi 4B (8GB) |
| **CPU** | 6-core ARM Cortex-A78AE @ 1.4GHz (Jetson) o ARM Cortex-A72 @ 1.8GHz (Pi 4B) |
| **RAM** | 8 GB LPDDR4 |
| **GPU** | NVIDIA GPU (128 CUDA cores) o VideoCore VI |
| **Almacenamiento** | 64-128 GB eMMC o NVMe SSD (recomendado SSD para mejor rendimiento) |
| **Conectividad** | Wi-Fi 802.11ac (dual-band), Ethernet Gigabit, USB 3.0 |
| **Sistema Operativo** | Ubuntu 22.04 LTS + ROS 2 Humble |

#### 2.1.3 Requisitos de Energía

- **Consumo típico**: 5-15 W (Raspberry Pi) o 10-25 W (Jetson)
- **Alimentación**: 5V/3A (Pi 4B) o 12V/2A (Jetson) con regulador de voltaje estable
- **Fuente de alimentación**: Debe ser compatible con el sistema de energía del robot

### 2.2 Interfaz con Controladores de Vuelo/Movimiento

La arquitectura se integra con controladores de vuelo (UAV) o controladores de movimiento (UGV) mediante ROS 2. No se requieren especificaciones del controlador físico, solo la interfaz ROS 2.

#### 2.2.1 UAV - Integración con Flight Controller

**Requisitos de Interfaz:**
- Comunicación serial (UART/USB) con flight controller (Pixhawk, Pixhawk 6C, o equivalente)
- Firmware compatible: **ArduPilot** (recomendado) o **PX4**
- Interfaz ROS 2 mediante **MAVROS** (ArduPilot) o **PX4 ROS 2 Interface**

**Configuración necesaria:**
- Permisos de puerto serial (`/dev/ttyACM0` o `/dev/ttyUSB0`)
- Configuración de baudrate (57600 o 115200)
- Usuario agregado al grupo `dialout`

#### 2.2.2 UGV - Integración con Controlador de Movimiento

**Requisitos de Interfaz:**
- Comunicación serial (UART/USB) o Ethernet con controlador de motores
- Interfaz ROS 2 para publicar comandos de velocidad (`/rover/cmd_vel`)
- Nodo ROS 2 que publique odometría (`/rover/odom`)

**Opciones de implementación:**
- Microcontrolador (Arduino/STM32) con nodo ROS 2 serial bridge
- Controlador comercial con soporte ROS 2
- Desarrollo de nodo ROS 2 personalizado

---

## 3. Requisitos de Sensores para la Arquitectura

### 3.1 UAV - Sensores Requeridos

#### 3.1.1 Sistema de Cámara

**Requisitos para Análisis con GPT-4o:**

| Parámetro | Especificación Mínima | Especificación Recomendada |
|-----------|----------------------|---------------------------|
| **Tipo** | Cámara RGB digital |
| **Resolución** | 640×480 @ 10 fps | 1920×1080 @ 30 fps |
| **Sensor** | 1/4" CMOS o mayor | 1/2.3" CMOS o mayor |
| **Lente** | FOV 60-90° | FOV 84° ajustable |
| **Orientación** | Nadir (hacia abajo) | Nadir con capacidad de inclinación ±30° |
| **Formato de Salida** | JPEG/MJPEG o RAW | H.264 hardware-accelerated |
| **Interfaz** | USB 2.0 o MIPI CSI-2 | USB 3.0, MIPI CSI-2, o Ethernet |

**Ejemplos de Cámaras Compatibles:**
- **Raspberry Pi Camera Module 3** (resolución media, bajo costo, MIPI CSI-2)
- **FLIR Blackfly S USB3** (alta resolución, industrial, USB 3.0)
- **Intel RealSense D435i** (RGB + profundidad, USB 3.0)
- **Logitech C920/C930e** (USB 2.0, buena calidad)

**Nota:** La arquitectura procesa imágenes a 256×256 píxeles internamente, pero una resolución mayor mejora la detección de objetos pequeños.

#### 3.1.2 GPS/GNSS

**Requisitos:**

| Parámetro | Especificación |
|-----------|----------------|
| **Tipo** | GPS + GLONASS + Galileo (multi-constelación) |
| **Precisión Horizontal** | < 5 m (GPS standalone), < 1 m (RTK opcional) |
| **Tasa de Actualización** | 1-10 Hz |
| **Tiempo de Adquisición** | < 60 segundos (frío), < 10 segundos (caliente) |
| **Interfaz** | UART (NMEA 0183) o USB |

**Integración:**
- Puede estar integrado en el flight controller (acceso vía MAVROS)
- O módulo GPS independiente conectado directamente a la computadora

**Topic ROS 2:** `/drone/gps/fix` (tipo `sensor_msgs/NavSatFix`)

#### 3.1.3 Sensor Inercial (IMU)

**Requisitos:**

| Parámetro | Especificación |
|-----------|----------------|
| **Tipo** | 9-DOF (Giroscopio 3-axis + Acelerómetro 3-axis + Magnetómetro 3-axis) |
| **Tasa de Muestreo** | 50-100 Hz (suficiente para arquitectura) |
| **Interfaz** | I2C, SPI, o UART |

**Integración:**
- Generalmente integrado en el flight controller (acceso vía MAVROS)
- O IMU independiente conectado a la computadora

**Topics ROS 2:**
- `/drone/imu` (tipo `sensor_msgs/Imu`)
- Odometría derivada: `/drone/odom` (tipo `nav_msgs/Odometry`)

### 3.2 UGV - Sensores Requeridos

#### 3.2.1 Sistema de Cámara

**Requisitos similares al UAV:**

| Parámetro | Especificación Mínima | Especificación Recomendada |
|-----------|----------------------|---------------------------|
| **Tipo** | Cámara RGB digital |
| **Resolución** | 640×480 @ 10 fps | 1920×1080 @ 30 fps |
| **Lente** | FOV 60-90° | FOV 84-120° (gran angular) |
| **Orientación** | Frontal | Frontal + cámara auxiliar trasera (opcional) |
| **Interfaz** | USB 2.0 o MIPI CSI-2 | USB 3.0 o Ethernet |

**Topic ROS 2:** `/rover/camera/image_raw` (tipo `sensor_msgs/Image`)

#### 3.2.2 Sensor de Rango (Obstáculos)

**Requisitos para Detección de Colisiones:**

**Opción 1: LiDAR 2D (Recomendado)**

| Parámetro | Especificación |
|-----------|----------------|
| **Tipo** | LiDAR 2D rotatorio |
| **Rango** | 4-12 m |
| **Resolución Angular** | 0.25-1.0° |
| **Tasa de Escaneo** | 5-20 Hz |
| **FOV Horizontal** | 360° o 270° |
| **Interfaz** | USB o Ethernet |

**Ejemplos:**
- **RPLIDAR A1/A2** (360°, USB, económico)
- **YDLIDAR X4** (360°, USB)
- **Hokuyo URG-04LX** (270°, USB)

**Opción 2: Sensores Ultrasónicos (Alternativa)**

- Rango: 2-4 m
- 4-8 sensores distribuidos alrededor del robot
- Tasa de actualización: 10-40 Hz
- Interfaz: GPIO, I2C, o UART

**Topic ROS 2:** `/rover/range` (tipo `sensor_msgs/Range`) o `/rover/scan` (tipo `sensor_msgs/LaserScan`)

#### 3.2.3 GPS/GNSS

**Requisitos similares al UAV** (opcional pero recomendado para navegación exterior).

**Topic ROS 2:** `/rover/gps/fix` (tipo `sensor_msgs/NavSatFix`)

#### 3.2.4 Sensor Inercial (IMU) y Odometría

**Requisitos:**

| Parámetro | Especificación |
|-----------|----------------|
| **IMU** | 9-DOF (opcional, para orientación) |
| **Odometría** | Encoders en ruedas o odometría visual |
| **Tasa de Muestreo** | 10-50 Hz |

**Topic ROS 2:** `/rover/odom` (tipo `nav_msgs/Odometry`)

---

## 4. Especificaciones de Software

### 4.1 Sistema Operativo

| Plataforma | Versión | Notas |
|-----------|---------|-------|
| **Ubuntu Linux** | 22.04 LTS (Jammy Jellyfish) | **Recomendado** - mejor soporte para ROS 2 |
| **ROS 2** | Humble Hawksbill | Framework principal de robótica |

**Alternativas:**
- Ubuntu 20.04 LTS con ROS 2 Foxy (compatible pero no recomendado)
- Debian 11 con ROS 2 Humble (posible con ajustes)

### 4.2 Stack de Software Principal

#### 4.2.1 Sistema Base ROS 2

**Instalación completa:**
```bash
ros-humble-desktop          # Instalación completa de ROS 2
ros-humble-cv-bridge        # Bridge OpenCV ↔ ROS Image
ros-humble-image-transport  # Transporte eficiente de imágenes
ros-humble-geometry-msgs    # Mensajes de geometría (Twist, Pose)
ros-humble-nav-msgs         # Mensajes de navegación (Odometry)
ros-humble-sensor-msgs      # Mensajes de sensores (Image, NavSatFix, Imu)
ros-humble-tf2              # Transformaciones de coordenadas
ros-humble-tf2-ros          # ROS 2 wrapper para TF2
ros-humble-std-msgs         # Mensajes estándar (String)
```

#### 4.2.2 Integración con Flight Controller (UAV)

**Para ArduPilot:**
```bash
ros-humble-mavros           # Interface ROS 2 ↔ MAVLink (ArduPilot)
ros-humble-mavros-extras    # Extras de MAVROS
```

**Para PX4:**
```bash
px4_ros_com                 # Interface ROS 2 para PX4
px4_msgs                    # Mensajes PX4
```

#### 4.2.3 Procesamiento de Imágenes

**OpenCV:**
```bash
libopencv-dev (v4.8.0+)     # Librerías de desarrollo
python3-opencv              # Bindings Python
```

**Librerías ROS 2 adicionales:**
```bash
cv_bridge                   # Conversión ROS Image ↔ OpenCV (incluido en ros-humble-cv-bridge)
image_transport             # Compresión y transporte de imágenes (incluido en ros-humble-image-transport)
```

#### 4.2.4 Inteligencia Artificial y Machine Learning

**LangGraph y LangChain:**
```bash
langgraph>=0.6.0            # Framework para workflows multi-agente
langchain-openai>=0.3.0     # Integración con OpenAI
langchain-core>=0.3.0       # Core de LangChain
```

**OpenAI API Client:**
```bash
openai>=1.0.0               # Cliente oficial de OpenAI
httpx>=0.24.0               # Cliente HTTP asíncrono
requests>=2.31.0            # Cliente HTTP síncrono
```

#### 4.2.5 Utilidades Python

```bash
numpy>=1.24.0               # Cálculos numéricos
python-dotenv>=1.0.0        # Gestión de variables de entorno
pydantic>=2.0.0             # Validación de datos
pillow>=10.0.0              # Procesamiento de imágenes
orjson>=3.9.0               # JSON rápido
anyio>=4.0.0                # Async I/O
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

# Multimedia (para procesamiento de video)
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

#### 4.3.2 Herramientas de Desarrollo ROS 2

```bash
python3-colcon-common-extensions  # Herramientas de build
python3-rosdep                    # Gestor de dependencias
python3-vcstool                   # Gestión de múltiples repositorios
```

### 4.4 Configuración de Red

#### 4.4.1 Requisitos de Conectividad

**Para cada robot (UAV/UGV):**
- Conexión Wi-Fi 802.11ac (dual-band) o Ethernet
- Acceso a Internet (para API de OpenAI GPT-4o)
- Opcional: Radio telemetría para comunicación inter-robot a larga distancia

**Estación Base (opcional, para monitoreo):**
- Conexión Ethernet o Wi-Fi estable
- Acceso a Internet
- Router/modem para comunicación con robots

#### 4.4.2 Configuración de Red ROS 2

**Variables de entorno:**
```bash
export ROS_DOMAIN_ID=0      # Usar mismo dominio para todos los nodos
export ROS_DISCOVERY_SERVER=  # Opcional: servidor de descubrimiento centralizado
```

**Configuración de IP estática (recomendado):**
```bash
# UAV
IP: 192.168.1.100
Netmask: 255.255.255.0
Gateway: 192.168.1.1

# UGV
IP: 192.168.1.101
Netmask: 255.255.255.0
Gateway: 192.168.1.1

# Estación Base (opcional)
IP: 192.168.1.1
```

### 4.5 Gestión de Configuración

#### 4.5.1 Variables de Entorno

**Archivo `.env` (en cada robot):**
```bash
# OpenAI API Key (REQUERIDO)
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
CAMERA_RESOLUTION_WIDTH=640
CAMERA_RESOLUTION_HEIGHT=480
CAMERA_FPS=10
GPS_RATE_HZ=10
```

#### 4.5.2 Archivos de Configuración YAML

**Configuración del escenario (`scenarios/default.yaml`):**
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
    width: 640
    height: 480
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
    Wi-Fi/Internet              Wi-Fi/Internet
         │                           │
         ▼                           ▼
┌─────────────────┐         ┌─────────────────┐
│      UAV        │◄───────►│      UGV        │
│                 │  ROS 2  │                 │
│                 │ Topics  │                 │
└─────────────────┘         └─────────────────┘
```

### 5.2 Protocolos de Comunicación

#### 5.2.1 ROS 2 Topics (Comunicación Principal)

**Topics del UAV:**
```bash
/drone/cmd_vel              # Comandos de velocidad (geometry_msgs/Twist)
/drone/odom                 # Odometría (nav_msgs/Odometry)
/drone/gps/fix              # Posición GPS (sensor_msgs/NavSatFix)
/drone/camera/image_raw     # Imagen de cámara (sensor_msgs/Image)
/drone/camera/camera_info   # Información de cámara (sensor_msgs/CameraInfo)
/uav/mission_brief          # Briefing de misión (std_msgs/String)
```

**Topics del UGV:**
```bash
/rover/cmd_vel              # Comandos de velocidad (geometry_msgs/Twist)
/rover/odom                 # Odometría (nav_msgs/Odometry)
/rover/gps/fix              # Posición GPS (sensor_msgs/NavSatFix)
/rover/range                # Sensor de rango (sensor_msgs/Range) o
/rover/scan                 # LiDAR scan (sensor_msgs/LaserScan)
/rover/camera/image_raw     # Imagen de cámara (sensor_msgs/Image)
```

**Topics de Comunicación Inter-Agente:**
```bash
/radio/uav_tx               # Mensajes del UAV (std_msgs/String)
/radio/uav_rx               # Mensajes recibidos por UAV (std_msgs/String)
/radio/ugv_tx               # Mensajes del UGV (std_msgs/String)
/radio/ugv_rx               # Mensajes recibidos por UGV (std_msgs/String)
```

#### 5.2.2 Formato de Mensajes

**Mensaje de Misión (JSON en `std_msgs/String`):**
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

#### 5.2.3 Radio Telemetría (Opcional)

Para comunicación inter-robot a larga distancia sin Internet:

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

**Nota:** La arquitectura puede funcionar completamente con Wi-Fi/Ethernet si los robots están en la misma red.

### 5.3 QoS y Confiabilidad

**Configuración QoS ROS 2:**
```python
# Comandos críticos (reliable)
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

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

## 6. Instalación y Configuración de la Arquitectura

### 6.1 Preparación del Sistema Operativo

#### 6.1.1 Instalación de Ubuntu 22.04 LTS

1. **Descargar imagen ISO** desde ubuntu.com (ARM64 para Raspberry Pi/Jetson)
2. **Crear medio booteable** (USB o microSD)
3. **Instalar en disco** (SSD recomendado para mejor rendimiento)
4. **Configurar particiones:**
   - `/` (root): 20-30 GB
   - `/home`: Resto del espacio
   - Swap: 2-4 GB

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
    ros-humble-tf2-ros \
    ros-humble-std-msgs

# Instalar MAVROS (solo para UAV con ArduPilot)
sudo apt install -y ros-humble-mavros ros-humble-mavros-extras
```

### 6.3 Instalación del Proyecto

#### 6.3.1 Clonar Repositorio

```bash
# Crear workspace ROS 2
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src

# Clonar proyecto
git clone https://github.com/renzoBC20/Disaster-multiagent-architecture-project.git
cd Disaster-multiagent-architecture-project
```

#### 6.3.2 Instalar Dependencias Python

```bash
# Crear entorno virtual
cd MultiAgent
python3 -m venv venv
source venv/bin/activate  # Linux/Mac
# venv\Scripts\activate   # Windows

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

### 6.4 Configuración de API Keys

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

### 6.5 Configuración de Red

#### 6.5.1 Configurar Wi-Fi (si es necesario)

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

### 6.6 Configuración de Servicios del Sistema

#### 6.6.1 Crear Servicio systemd para Nodos ROS 2

**Archivo: `/etc/systemd/system/uav-controller.service`** (para UAV)
```ini
[Unit]
Description=UAV LangGraph Controller
After=network.target

[Service]
Type=simple
User=robot  # Cambiar por tu usuario
WorkingDirectory=/home/robot/robot_ws
Environment="ROS_DOMAIN_ID=0"
Environment="PATH=/home/robot/robot_ws/src/Disaster-multiagent-architecture-project/MultiAgent/venv/bin:/usr/bin:/bin"
ExecStart=/home/robot/robot_ws/src/Disaster-multiagent-architecture-project/MultiAgent/venv/bin/python3 /home/robot/robot_ws/src/Disaster-multiagent-architecture-project/robotic-ai-agents/simulator/microsim/scripts/uav_langgraph_controller.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

**Archivo: `/etc/systemd/system/ugv-controller.service`** (para UGV)
```ini
[Unit]
Description=UGV LangGraph Controller
After=network.target

[Service]
Type=simple
User=robot
WorkingDirectory=/home/robot/robot_ws
Environment="ROS_DOMAIN_ID=0"
Environment="PATH=/home/robot/robot_ws/src/Disaster-multiagent-architecture-project/MultiAgent/venv/bin:/usr/bin:/bin"
ExecStart=/home/robot/robot_ws/src/Disaster-multiagent-architecture-project/MultiAgent/venv/bin/python3 /home/robot/robot_ws/src/Disaster-multiagent-architecture-project/robotic-ai-agents/simulator/microsim/scripts/ugv_langgraph_controller.py
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

**Habilitar servicio:**
```bash
sudo systemctl daemon-reload
sudo systemctl enable uav-controller.service  # o ugv-controller.service
sudo systemctl start uav-controller.service
```

---

## 7. Configuración de Integración con Robots

### 7.1 UAV - Integración con Flight Controller

#### 7.1.1 Configurar MAVROS (ArduPilot)

**Configurar conexión serial:**
```bash
# Agregar usuario a grupo dialout
sudo usermod -a -G dialout $USER

# Configurar udev rules para Pixhawk (ajustar según tu hardware)
sudo bash -c 'cat > /etc/udev/rules.d/99-pixhawk.rules << EOF
SUBSYSTEM=="tty", ATTRS{idVendor}=="26ac", ATTRS{idProduct}=="0011", MODE="0666", GROUP="dialout"
EOF'
sudo udevadm control --reload-rules

# Reiniciar sesión para aplicar cambios de grupo
```

**Ejecutar MAVROS:**
```bash
# Identificar puerto serial
ls /dev/ttyACM* /dev/ttyUSB*

# Ejecutar MAVROS
ros2 run mavros mavros_node --ros-args \
    -p fcu_url:=/dev/ttyACM0:57600 \
    -p system_id:=1 \
    -p component_id:=191
```

**Verificar conexión:**
```bash
# Ver estado del flight controller
ros2 topic echo /mavros/state

# Ver odometría
ros2 topic echo /mavros/global_position/local

# Ver GPS
ros2 topic echo /mavros/global_position/global
```

#### 7.1.2 Configurar Bridge de Topics

La arquitectura espera los siguientes topics. Si tu flight controller usa nombres diferentes, crea un nodo bridge:

**Topics requeridos:**
- `/drone/cmd_vel` (geometry_msgs/Twist)
- `/drone/odom` (nav_msgs/Odometry)
- `/drone/gps/fix` (sensor_msgs/NavSatFix)
- `/drone/camera/image_raw` (sensor_msgs/Image)

**Ejemplo de bridge (crear nodo ROS 2 personalizado):**
```python
# mavros_bridge.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix

class MAVROSBridge(Node):
    def __init__(self):
        super().__init__('mavros_bridge')
        # Subscribirse a topics de MAVROS
        self.sub_cmd = self.create_subscription(
            Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped',
            self.cmd_callback, 10)
        # ... más subscriptions

        # Publicar a topics esperados por la arquitectura
        self.pub_cmd = self.create_publisher(Twist, '/drone/cmd_vel', 10)
        # ... más publishers
```

### 7.2 UGV - Integración con Controlador de Movimiento

#### 7.2.1 Configurar Comunicación Serial

**Identificar puerto:**
```bash
lsusb
dmesg | grep tty

# Configurar permisos
sudo chmod 666 /dev/ttyUSB0  # o ttyACM0
```

**Crear nodo ROS 2 serial bridge:**
- Ejemplo usando `pyserial` y ROS 2
- Leer comandos desde `/rover/cmd_vel`
- Publicar odometría en `/rover/odom`
- Publicar datos de sensores en topics correspondientes

#### 7.2.2 Configurar Sensores

**Cámara:**
```bash
# Verificar detección
lsusb | grep -i camera
v4l2-ctl --list-devices

# Probar captura
v4l2-ctl --device=/dev/video0 --stream-mmap --stream-count=1 --stream-to=test.raw
```

**LiDAR:**
```bash
# Verificar detección
lsusb | grep -i lidar

# Probar con driver del fabricante (ej: rplidar_ros)
ros2 launch rplidar_ros rplidar.launch.py
```

**GPS:**
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

### 7.3 Configuración de Transformaciones (TF)

La arquitectura requiere un árbol de transformaciones publicado:

```bash
# Verificar TF tree
ros2 run tf2_tools view_frames

# Topics requeridos:
# /tf (tf2_msgs/TFMessage)
# /tf_static (tf2_msgs/TFMessage)
```

**Frames requeridos:**
- `map` → `odom` → `drone/base_link` o `rover/base_link`
- `drone/base_link` → `drone/camera_link` (UAV)
- `rover/base_link` → `rover/camera_link` (UGV)

---

## 8. Testing y Validación

### 8.1 Testing de Software

#### 8.1.1 Pruebas Unitarias

```bash
cd ~/robot_ws
colcon test --packages-select microsim
colcon test-result --verbose
```

#### 8.1.2 Pruebas de Integración con Simulador

```bash
# Terminal 1: Iniciar simulador
source ~/robot_ws/install/setup.bash
ros2 run microsim microsim_node

# Terminal 2: Ejecutar controlador UAV
source ~/robot_ws/install/setup.bash
cd ~/robot_ws/src/Disaster-multiagent-architecture-project/MultiAgent
source venv/bin/activate
python3 ../robotic-ai-agents/simulator/microsim/scripts/uav_langgraph_controller.py
```

### 8.2 Testing con Hardware Real

#### 8.2.1 Verificar Sensores

```bash
# Ver topics disponibles
ros2 topic list

# Ver datos de GPS
ros2 topic echo /drone/gps/fix

# Ver imágenes de cámara
ros2 topic echo /drone/camera/image_raw --no-arr

# Ver odometría
ros2 topic echo /drone/odom
```

#### 8.2.2 Verificar Comunicación

```bash
# Ver nodos activos
ros2 node list

# Ver información de un nodo
ros2 node info /uav_langgraph_controller

# Verificar comunicación inter-robot
ros2 topic echo /radio/uav_tx
```

### 8.3 Pruebas de Campo

#### 8.3.1 Fase 1: Pruebas Básicas

1. **Verificar publicación de topics** (GPS, cámara, odometría)
2. **Probar comando de movimiento** (publicar en `/drone/cmd_vel` o `/rover/cmd_vel`)
3. **Verificar captura de imágenes**
4. **Probar comunicación inter-agente**

#### 8.3.2 Fase 2: Pruebas Autónomas

1. **Ejecutar workflow completo de reconocimiento** (UAV)
2. **Probar detección de víctimas/obstáculos**
3. **Verificar generación de briefing de misión**
4. **Probar recepción y ejecución de misión** (UGV)

#### 8.3.3 Fase 3: Pruebas de Misión Completa

1. **Reconocimiento aéreo completo**
2. **Identificación de víctimas/obstáculos con GPT-4o**
3. **Planificación de ruta optimizada**
4. **Ejecución de rescate terrestre**

### 8.4 Métricas de Validación

| Métrica | Objetivo | Método de Medición |
|---------|----------|-------------------|
| **Tasa de Publicación de Topics** | Según configuración (10 Hz GPS, 10 Hz cámara) | `ros2 topic hz /topic_name` |
| **Latencia de Comandos** | < 100 ms | Timestamps de comandos vs ejecución |
| **Tasa de Detección de Víctimas** | > 90% | Comparar con ground truth |
| **Tasa de Falsos Positivos** | < 5% | Análisis manual |
| **Tiempo de Comunicación Inter-Agente** | < 500 ms | Timestamps de mensajes |
| **Uptime del Sistema** | > 95% | Logs del sistema |

---

## 9. Troubleshooting

### 9.1 Problemas de Instalación

#### Problema: ROS 2 no se encuentra

**Diagnóstico:**
```bash
echo $ROS_DISTRO
source /opt/ros/humble/setup.bash
```

**Solución:**
- Verificar instalación de ROS 2
- Agregar `source /opt/ros/humble/setup.bash` a `~/.bashrc`

#### Problema: Dependencias Python faltantes

**Diagnóstico:**
```bash
cd ~/robot_ws/src/Disaster-multiagent-architecture-project/MultiAgent
source venv/bin/activate
pip list
```

**Solución:**
```bash
pip install -r requirements.txt
```

### 9.2 Problemas de Comunicación

#### Problema: Robots no se comunican

**Diagnóstico:**
```bash
# Verificar red
ping 192.168.1.100  # IP del otro robot

# Verificar ROS 2 discovery
ros2 node list
```

**Soluciones:**
- Verificar mismo `ROS_DOMAIN_ID`
- Revisar configuración de red (IPs, máscara)
- Verificar firewall: `sudo ufw status`
- Revisar conexión Wi-Fi/Ethernet

#### Problema: Topics no aparecen

**Diagnóstico:**
```bash
ros2 topic list
ros2 topic echo /drone/odom
```

**Soluciones:**
- Verificar que los nodos estén ejecutándose
- Verificar nombres de topics (case-sensitive)
- Revisar logs: `ros2 node info /node_name`

### 9.3 Problemas de Sensores

#### Problema: Cámara no funciona

**Diagnóstico:**
```bash
lsusb | grep -i camera
v4l2-ctl --list-devices
ros2 topic echo /drone/camera/image_raw
```

**Soluciones:**
- Verificar conexión USB/cable
- Instalar drivers del fabricante
- Verificar permisos: `sudo usermod -a -G video $USER`
- Verificar configuración del nodo de cámara

#### Problema: GPS no publica datos

**Diagnóstico:**
```bash
ros2 topic echo /drone/gps/fix
cgps -s  # Si está disponible
```

**Soluciones:**
- Verificar conexión del módulo GPS
- Mover a área abierta (mejor recepción)
- Esperar > 60 segundos para adquisición inicial
- Verificar configuración de baudrate

### 9.4 Problemas de Software

#### Problema: Nodos no inician

**Diagnóstico:**
```bash
# Ver errores del servicio
journalctl -u uav-controller.service -n 50

# Verificar dependencias
rosdep check --from-paths src --ignore-src
```

**Soluciones:**
- Verificar instalación de dependencias
- Revisar configuración de servicios systemd
- Verificar permisos de archivos
- Revisar logs de errores: `ros2 run <package> <node> --ros-args --log-level debug`

#### Problema: Error de API de OpenAI

**Diagnóstico:**
```bash
# Verificar API key
cat ~/robot_ws/src/Disaster-multiagent-architecture-project/MultiAgent/.env
```

**Soluciones:**
- Verificar que la API key esté configurada correctamente
- Verificar conectividad a Internet
- Verificar cuota/balance de cuenta de OpenAI
- Revisar logs de errores del nodo

---

## 10. Referencias

### 10.1 Documentación Oficial

- **ROS 2 Documentation**: https://docs.ros.org/en/humble/
- **ArduPilot Documentation**: https://ardupilot.org/
- **PX4 Documentation**: https://docs.px4.io/
- **OpenCV Documentation**: https://docs.opencv.org/
- **LangGraph Documentation**: https://python.langchain.com/docs/langgraph
- **OpenAI API Documentation**: https://platform.openai.com/docs

### 10.2 Recursos Adicionales

- **Repositorio del Proyecto**: https://github.com/renzoBC20/Disaster-multiagent-architecture-project
- **Issues y Soporte**: https://github.com/renzoBC20/Disaster-multiagent-architecture-project/issues
- **ROS 2 Tutorials**: https://docs.ros.org/en/humble/Tutorials.html
- **MAVROS Documentation**: http://wiki.ros.org/mavros

### 10.3 Glosario de Términos

- **AGL**: Above Ground Level (altura sobre el nivel del suelo)
- **FOV**: Field of View (campo de visión)
- **GNSS**: Global Navigation Satellite System
- **IMU**: Inertial Measurement Unit
- **MAVLink**: Protocolo de comunicación para vehículos aéreos autónomos
- **RTK**: Real-Time Kinematic (GPS de alta precisión)
- **SLAM**: Simultaneous Localization and Mapping
- **TF**: Transform Frame (sistema de transformaciones de coordenadas en ROS)

---

**Versión del Documento:** 1.0  
**Última Actualización:** 2025  
**Estado:** Documento de Referencia

---

**Autor:** Renzo BC20  
**Contacto:** https://github.com/renzoBC20
