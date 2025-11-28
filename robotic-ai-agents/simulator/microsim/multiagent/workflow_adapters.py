"""
Adaptadores de funciones del workflow de LangGraph para ROS 2.

Estas funciones adaptan las funciones originales de MultiAgent para que
funcionen con datos de ROS 2 en lugar de video estático.
"""

import os
import sys
import cv2
import base64
import json
import time
import math
from typing import Dict, Any
import numpy as np

# Importar funciones base de MultiAgent
try:
    multiagent_path = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "MultiAgent")
    if os.path.exists(multiagent_path):
        sys.path.insert(0, multiagent_path)
    MULTIAGENT_AVAILABLE = True
except:
    MULTIAGENT_AVAILABLE = False


def procesadorVideo_ros2(state: Dict[str, Any]) -> Dict[str, Any]:
    """
    Adaptación de procesadorVideo para ROS 2.
    Si tenemos video_cap, lee del video grabado.
    Si no, usa la imagen ya almacenada en el estado.
    """
    # Si tenemos video_cap, usar la función original de procesadorVideo
    if state.get("video_cap") is not None:
        from UAV_agent import procesadorVideo
        return procesadorVideo(state)
    
    # Modo ROS 2 live: usar imagen ya almacenada
    if "latest_camera_image" not in state:
        print("⚠️ No hay imagen disponible todavía")
        return state
    
    try:
        # La imagen ya viene procesada desde adapt_uav_state_for_ros2
        if state.get("frame_data") is not None and state.get("frame_base64"):
            # El frame ya está codificado
            print(f"🎬 Frame {state.get('current_frame', 0) + 1} procesado")
            state["current_frame"] = state.get("current_frame", 0) + 1
        else:
            print("⚠️ Frame sin datos")
    except Exception as e:
        print(f"⚠️ Error procesando frame ROS 2: {e}")
    
    return state


def inicializadorPatrullaje_ros2(state: Dict[str, Any]) -> Dict[str, Any]:
    """
    Adaptación de inicializadorPatrullaje para ROS 2.
    Genera ruta de patrullaje usando dimensiones del mundo de MicroSim.
    """
    print("🛸 UAV: Inicializando sistema de patrullaje para ROS 2...")
    
    # Usar dimensiones del mundo de MicroSim (100x100 metros)
    world_width = 100.0  # metros
    world_height = 100.0
    world_resolution = 256  # resolución de imagen de la cámara
    
    # Generar ruta circular que cubre toda la zona
    patrol_route = _generate_circular_patrol_route_ros2(
        world_width, world_height,
        start_pos=state.get("uav_position", (0, 0))
    )
    
    state["patrol_route"] = patrol_route
    state["patrol_index"] = 0
    state["patrol_complete"] = False
    state["movement_speed"] = 3.0  # metros por comando
    
    print(f"🗺️ UAV: Ruta de patrullaje generada")
    print(f"   └─ Área de cobertura: {world_width}x{world_height} metros")
    print(f"   └─ Puntos de patrullaje: {len(patrol_route)}")
    
    return state


def _generate_circular_patrol_route_ros2(width: float, height: float, start_pos: tuple) -> list:
    """
    Genera una ruta circular que cubre el perímetro de la zona (en coordenadas del mundo).
    
    Args:
        width: Ancho del mundo en metros
        height: Alto del mundo en metros
        start_pos: Posición inicial (x, y) en metros
        
    Returns:
        Lista de puntos (x, y) en metros para la ruta de patrullaje
    """
    route = []
    margin = 10.0  # Margen desde los bordes en metros
    
    # Esquina superior izquierda
    route.append((-width/2 + margin, height/2 - margin))
    
    # Lado superior (izquierda a derecha)
    for x in range(int(-width/2 + margin), int(width/2 - margin), 10):
        route.append((x, height/2 - margin))
    route.append((width/2 - margin, height/2 - margin))
    
    # Lado derecho (arriba a abajo)
    for y in range(int(height/2 - margin), int(-height/2 + margin), -10):
        route.append((width/2 - margin, y))
    route.append((width/2 - margin, -height/2 + margin))
    
    # Lado inferior (derecha a izquierda)
    for x in range(int(width/2 - margin), int(-width/2 + margin), -10):
        route.append((x, -height/2 + margin))
    route.append((-width/2 + margin, -height/2 + margin))
    
    # Lado izquierdo (abajo a arriba)
    for y in range(int(-height/2 + margin), int(height/2 - margin), 10):
        route.append((-width/2 + margin, y))
    
    # Volver al punto de inicio
    route.append(start_pos)
    
    return route


def controladorPatrullaje_ros2(state: Dict[str, Any]) -> Dict[str, Any]:
    """
    Adaptación de controladorPatrullaje para ROS 2.
    NOTA: Este nodo realmente controla el movimiento, pero en ROS 2
    eso se hace desde el control loop. Aquí solo marcamos el patrullaje como completo.
    """
    # Para ROS 2, el control del movimiento se maneja en el control loop
    # Este nodo solo marca el progreso
    patrol_index = state.get("patrol_index", 0)
    patrol_route = state.get("patrol_route", [])
    
    if patrol_index >= len(patrol_route):
        state["patrol_complete"] = True
        print("✅ Patrullaje completado")
    
    return state


def cargadorMensaje_ros2(state: Dict[str, Any]) -> Dict[str, Any]:
    """
    Adaptación de cargadorMensaje para ROS 2.
    En lugar de guardar en archivo JSON, prepara el mensaje para publicar en ROS 2.
    """
    current_frame = state.get("current_frame", 0)
    max_frames = state.get("max_frames_to_process", 1)
    
    # Solo cargar mensaje en el último frame
    if current_frame >= max_frames:
        mission_brief = state.get("mission_brief", "")
        
        if mission_brief:
            print("📤 UAV: Mensaje preparado para publicar en ROS 2")
            
            # El mensaje se publicará desde el controlador ROS 2
            state["mission_ready_to_publish"] = True
        else:
            print("⚠️ UAV: No hay mensaje compilado para cargar")
    
    return state

