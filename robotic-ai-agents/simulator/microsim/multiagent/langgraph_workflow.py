"""
Integración completa de LangGraph con ROS 2.
Este módulo integra los workflows completos de StateGraph de MultiAgent
con el sistema ROS 2, adaptando el flujo para tiempo real.
"""

import os
import sys
import json
import time
from typing import Dict, Any
import numpy as np

# Importar LangGraph y LangChain
try:
    from langgraph.graph import StateGraph, END
    from langchain_openai import ChatOpenAI
    from dotenv import load_dotenv
    
    # Cargar .env desde MultiAgent (desde microsim/multiagent → MultiAgent es ../../..\MultiAgent)
    multiagent_env_path = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "MultiAgent", ".env")
    load_dotenv(multiagent_env_path)
    
    llm = ChatOpenAI(model="gpt-5", api_key=os.getenv("OPENAI_API_KEY"))
    LANGGRAPH_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ Advertencia: LangGraph no disponible: {e}")
    llm = None
    StateGraph = None
    END = None
    LANGGRAPH_AVAILABLE = False

# Importar funciones de MultiAgent
try:
    multiagent_path = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "MultiAgent")
    if os.path.exists(multiagent_path):
        sys.path.insert(0, multiagent_path)
        from UAV_agent import (
            procesadorVideo,
            identificacionVictimas,
            identificacionObstaculos,
            planificacionRuta,
            briefingMision,
            cargadorMensaje,
            State as UAVState,
            encode_frame_to_base64,
            create_uav_graph  # Importar el workflow original
        )
        from UGV_Agent import (
            receptorMensaje,
            planificadorMision,
            ejecutorMision,
            controladorMovimiento,
            atencionVictimas,
            continuarEjecutor,
            continuarControlador,
            UGVState
        )
        MULTIAGENT_AVAILABLE = True
except ImportError as e:
    print(f"⚠️ Advertencia: MultiAgent modules no disponibles: {e}")
    MULTIAGENT_AVAILABLE = False


def create_uav_workflow_for_ros2():
    """
    Crea el workflow de LangGraph para UAV usando el workflow ORIGINAL de UAV_agent.py.
    
    El workflow original:
    1. procesadorVideo - toma frame del video, lo codifica en base64
    2. identificacionVictimas - identifica víctimas
    3. identificacionObstaculos - identifica obstáculos
    4. planificacionRuta - planifica ruta
    5. briefingMision - genera briefing
    6. cargadorMensaje - carga el mensaje
    
    Returns:
        Compiled workflow original o None si no está disponible
    """
    if not LANGGRAPH_AVAILABLE or not MULTIAGENT_AVAILABLE:
        return None
    
    try:
        # Usar el workflow original de UAV_agent.py directamente
        # Este workflow ya tiene el flujo correcto sin adaptaciones
        original_workflow = create_uav_graph()
        
        print("✅ Usando workflow original de UAV_agent.py")
        print("   Flujo: procesadorVideo → identificacionVictimas → identificacionObstaculos → planificacionRuta → briefingMision → cargadorMensaje")
        
        return original_workflow
        
    except Exception as e:
        print(f"❌ Error usando workflow original: {e}")
        import traceback
        traceback.print_exc()
        return None


def create_ugv_workflow_for_ros2():
    """
    Crea el workflow de LangGraph para UGV adaptado para ROS 2.
    
    Returns:
        Compiled workflow o None si no está disponible
    """
    if not LANGGRAPH_AVAILABLE or not MULTIAGENT_AVAILABLE:
        return None
    
    try:
        # Crear grafo
        ugv_workflow = StateGraph(UGVState)
        
        # Agregar nodos
        ugv_workflow.add_node("receptorMensaje", receptorMensaje)
        ugv_workflow.add_node("planificadorMision", planificadorMision)
        ugv_workflow.add_node("ejecutorMision", ejecutorMision)
        ugv_workflow.add_node("controladorMovimiento", controladorMovimiento)
        ugv_workflow.add_node("atencionVictimas", atencionVictimas)
        
        # Definir flujo
        ugv_workflow.set_entry_point("receptorMensaje")
        ugv_workflow.add_edge("receptorMensaje", "planificadorMision")
        ugv_workflow.add_edge("planificadorMision", "ejecutorMision")
        
        # Flujo entre ejecutor y controlador
        ugv_workflow.add_conditional_edges(
            "ejecutorMision",
            continuarEjecutor,
            {
                "ejecutar": "ejecutorMision",
                "mover": "controladorMovimiento",
                "finalizar": END
            }
        )
        
        # Flujo del controlador
        ugv_workflow.add_conditional_edges(
            "controladorMovimiento",
            continuarControlador,
            {
                "mover": "controladorMovimiento",
                "atender": "atencionVictimas",
                "ejecutar": "ejecutorMision"
            }
        )
        
        # Flujo de atención
        ugv_workflow.add_edge("atencionVictimas", "ejecutorMision")
        
        return ugv_workflow.compile()
        
    except Exception as e:
        print(f"❌ Error creando workflow UGV: {e}")
        return None


def adapt_uav_state_for_ros2(ros2_state: Dict[str, Any], camera_image=None):
    """
    Convierte el estado de ROS 2 al formato State de LangGraph.
    
    Args:
        ros2_state: Estado del controlador ROS 2
        camera_image: Imagen de la cámara ROS 2 (opcional)
        
    Returns:
        Estado adaptado para LangGraph
    """
    if not MULTIAGENT_AVAILABLE:
        return {}
    
    # Si tenemos video_cap, usar video grabado en lugar de imagen en vivo
    video_cap = ros2_state.get("video_cap")
    video_path = ros2_state.get("video_path", "ros2_live")
    
    if video_cap is not None:
        # Usar video grabado - el workflow leerá los frames del video
        adapted_state = {
            "video_path": video_path,
            "current_frame": ros2_state.get("current_frame", 0),
            "total_frames": ros2_state.get("total_frames", 1),
            "max_frames_to_process": ros2_state.get("max_frames_to_process", 5),
            "frame_data": None,  # Se llenará por procesadorVideo
            "frame_base64": "",
            "analysis_result": "",
            "video_cap": video_cap,  # Pasar el video_cap para que se use
            "victims_found": ros2_state.get("detected_victims", []),
            "obstacles_found": ros2_state.get("detected_obstacles", []),
            "routes_planned": ros2_state.get("planned_routes", []),
            "mission_brief": ros2_state.get("mission_brief", ""),
            "communication_log": ros2_state.get("communication_log", []),
            "uav_position": (ros2_state.get("position", [0, 0, 0])[0:2] if isinstance(ros2_state.get("position"), np.ndarray) else (0, 0)),
            "patrol_route": ros2_state.get("patrol_route", []),
            "patrol_index": ros2_state.get("patrol_index", 0),
            "patrol_complete": ros2_state.get("patrol_complete", False),
            "movement_speed": ros2_state.get("movement_speed", 3.0),
            "patrol_initialized": True
        }
        return adapted_state
    
    # Modo normal: usar imagen en vivo de ROS 2
    adapted_state = {
        "video_path": "ros2_live",
        "current_frame": ros2_state.get("frame_counter", 0),
        "total_frames": ros2_state.get("max_frames", 1),
        "max_frames_to_process": ros2_state.get("max_frames", 1),
        "frame_data": None,
        "frame_base64": "",
        "analysis_result": "",
        "video_cap": None,
        "victims_found": ros2_state.get("detected_victims", []),
        "obstacles_found": ros2_state.get("detected_obstacles", []),
        "routes_planned": ros2_state.get("planned_routes", []),
        "mission_brief": ros2_state.get("mission_brief", ""),
        "communication_log": ros2_state.get("communication_log", []),
        "uav_position": (ros2_state.get("position", [0, 0, 0])[0:2] if isinstance(ros2_state.get("position"), np.ndarray) else (0, 0)),
        "patrol_route": ros2_state.get("patrol_route", []),
        "patrol_index": ros2_state.get("patrol_index", 0),
        "patrol_complete": ros2_state.get("patrol_complete", False),
        "movement_speed": ros2_state.get("movement_speed", 3.0)
    }
    
    # Procesar imagen de ROS 2 si está disponible
    if camera_image is not None:
        try:
            import cv2
            from cv_bridge import CvBridge
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(camera_image, desired_encoding='rgb8')
            
            # Codificar
            encoded = encode_frame_to_base64(cv_image, adapted_state["current_frame"])
            
            adapted_state["frame_data"] = cv_image
            adapted_state["frame_base64"] = encoded
            
        except Exception as e:
            print(f"⚠️ Error adaptando imagen: {e}")
    
    return adapted_state


def extract_ros2_state_from_langgraph(langgraph_state: Dict[str, Any]):
    """
    Extrae información relevante del estado de LangGraph para ROS 2.
    
    Args:
        langgraph_state: Estado de LangGraph después de ejecutar workflow
        
    Returns:
        Estado actualizado para ROS 2
    """
    return {
        "detected_victims": langgraph_state.get("victims_found", []),
        "detected_obstacles": langgraph_state.get("obstacles_found", []),
        "planned_routes": langgraph_state.get("routes_planned", []),
        "mission_brief": langgraph_state.get("mission_brief", ""),
        "communication_log": langgraph_state.get("communication_log", []),
        "patrol_complete": langgraph_state.get("patrol_complete", False),
        "patrol_route": langgraph_state.get("patrol_route", []),
        "patrol_index": langgraph_state.get("patrol_index", 0),
        "current_frame": langgraph_state.get("current_frame", 0)
    }


def adapt_ugv_state_for_ros2(ros2_state: Dict[str, Any], mission_data=None):
    """
    Convierte el estado de ROS 2 UGV al formato UGVState de LangGraph.
    
    Args:
        ros2_state: Estado del controlador UGV ROS 2
        mission_data: Datos de misión recibidos (opcional)
        
    Returns:
        Estado adaptado para LangGraph
    """
    if not MULTIAGENT_AVAILABLE:
        return {}
    
    adapted_state = {
        "mission_received": ros2_state.get("mission_received", False),
        "mission_data": mission_data or ros2_state.get("mission_data", {}),
        "current_position": tuple(ros2_state.get("position", [0, 0])[:2]) if isinstance(ros2_state.get("position"), np.ndarray) else (0, 0),
        "target_victims": ros2_state.get("target_victims", []),
        "obstacles_avoided": ros2_state.get("obstacles_avoided", []),
        "mission_status": ros2_state.get("mission_status", "WAITING"),
        "communication_log": ros2_state.get("communication_log", []),
        "message_file": "ros2_topic",  # Indica que viene de ROS 2, no de archivo
        "target_point": ros2_state.get("target_point"),
        "route_index": ros2_state.get("route_index", 0),
        "arrival_confirmed": ros2_state.get("arrival_confirmed", False),
        "movement_speed": ros2_state.get("movement_speed", 2.0),
        "current_victim_info": ros2_state.get("current_victim_info", {}),
        "attention_in_progress": ros2_state.get("attention_in_progress", False),
        "attention_complete": ros2_state.get("attention_complete", False),
        "victims_rescued": ros2_state.get("victims_rescued", [])
    }
    
    return adapted_state


def extract_ugv_ros2_state_from_langgraph(langgraph_state: Dict[str, Any]):
    """
    Extrae información relevante del estado de LangGraph para UGV ROS 2.
    
    Args:
        langgraph_state: Estado de LangGraph después de ejecutar workflow
        
    Returns:
        Estado actualizado para ROS 2
    """
    return {
        "mission_received": langgraph_state.get("mission_received", False),
        "mission_data": langgraph_state.get("mission_data", {}),
        "target_victims": langgraph_state.get("target_victims", []),
        "obstacles_avoided": langgraph_state.get("obstacles_avoided", []),
        "mission_status": langgraph_state.get("mission_status", "WAITING"),
        "communication_log": langgraph_state.get("communication_log", []),
        "target_point": langgraph_state.get("target_point"),
        "route_index": langgraph_state.get("route_index", 0),
        "arrival_confirmed": langgraph_state.get("arrival_confirmed", False),
        "current_victim_info": langgraph_state.get("current_victim_info", {}),
        "attention_in_progress": langgraph_state.get("attention_in_progress", False),
        "attention_complete": langgraph_state.get("attention_complete", False),
        "victims_rescued": langgraph_state.get("victims_rescued", [])
    }

