#!/usr/bin/env python3
"""
Controlador UAV con Integración Completa de LangGraph

Este controlador integra el workflow completo de StateGraph de LangGraph
del sistema MultiAgent con ROS 2, respetando la arquitectura original.

Autor: Integración MultiAgent-MicroSim
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import math
import json
import time
import sys
import os
import cv2
import re
from cv_bridge import CvBridge

# Agregar path al módulo multiagent
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from multiagent.langgraph_workflow import (
    create_uav_workflow_for_ros2,
    adapt_uav_state_for_ros2,
    extract_ros2_state_from_langgraph
)
from multiagent.coordinate_transforms import CoordinateTransformer
from scripts.autonomous_drone_controller import AutonomousDroneController


class UAVLangGraphController(AutonomousDroneController):
    """
    Controlador UAV que usa el workflow completo de LangGraph.
    
    Respeta la arquitectura original de MultiAgent donde:
    - Patrullaje inicial → Procesamiento de frames → Análisis → Planificación
    - StateGraph maneja el flujo de estado
    - Cada nodo actualiza el estado compartido
    """
    
    def __init__(self):
        super().__init__()
        
        # ====================================================================
        # CONFIGURACIÓN
        # ====================================================================
        self.enable_langgraph = True  # Usar workflow completo de LangGraph
        self.workflow = None
        self.langgraph_state = {}  # Estado compartido de LangGraph
        
        # ====================================================================
        # ESTADO DEL AGENTE LANGGRAPH
        # ====================================================================
        self.frame_counter = 0
        self.patrol_complete = False
        self.processing_complete = False
        
        # Video recording state
        self.recording_video = False
        self.video_writer = None
        self.video_frames = []
        self.video_frames_metadata = []  # Metadata de cada frame del video
        self.video_fps = 2.0  # 2 fps para captura periódica
        self.video_duration = 5.0  # 5 segundos de video
        self.video_start_time = None
        self.waypoint_reached_triggered = False
        self.frame_skip_counter = 0  # Para limitar captura a 2 fps
        
        # Metadata del frame actual (posición del UAV cuando se capturó)
        self.current_frame_metadata = {
            "uav_x": 0.0,
            "uav_y": 0.0,
            "uav_z": 0.0,
            "uav_yaw": 0.0
        }
        
        # Transformador de coordenadas
        # world_size=100.0 porque el mundo va de -50 a +50 metros (100 metros total)
        self.coord_transformer = CoordinateTransformer(
            image_width=256,
            image_height=256,
            world_size=100.0
        )
        
        # CV Bridge para conversión de imágenes ROS → OpenCV
        self.cv_bridge = CvBridge()
        
        # ====================================================================
        # ROS 2 PUBLISHERS
        # ====================================================================
        self.mission_pub = self.create_publisher(
            String, '/uav/mission_brief', 10
        )
        
        # ====================================================================
        # INICIALIZAR LANGGRAPH WORKFLOW
        # ====================================================================
        self._initialize_langgraph_workflow()
        
        # ====================================================================
        # TIMERS PARA EJECUTAR WORKFLOW
        # ====================================================================
        # Timer para patrullaje (más lento)
        self.patrol_timer = self.create_timer(0.5, self.patrol_loop)
        
        # Timer para procesamiento (más rápido, pero solo cuando patrol_complete)
        self.processing_timer = self.create_timer(5.0, self.processing_loop)
        
        self.get_logger().info('🚁 UAV LangGraph Controller inicializado!')
        self.get_logger().info(f'   LangGraph workflow: {"✅ Activo" if self.workflow else "❌ No disponible"}')
        
        # IMPORTANTE: Iniciar misión automáticamente para que el dron se mueva
        self.start_mission()
        self.get_logger().info('🚀 Misión iniciada automáticamente')
    
    def _initialize_langgraph_workflow(self):
        """
        Inicializa el workflow de LangGraph.
        """
        try:
            self.workflow = create_uav_workflow_for_ros2()
            
            if self.workflow:
                self.get_logger().info('✅ Workflow de LangGraph compilado exitosamente')
                
                # Inicializar estado de LangGraph
                self.langgraph_state = {
                    "video_path": "ros2_live",
                    "current_frame": 0,
                    "total_frames": 1,
                    "max_frames_to_process": 1,  # Procesar solo 1 frame
                    "frame_data": None,
                    "frame_base64": "",
                    "analysis_result": "",
                    "video_cap": None,  # Simulado para ROS 2
                    "victims_found": [],
                    "obstacles_found": [],
                    "routes_planned": [],
                    "mission_brief": "",
                    "communication_log": [],
                    "uav_position": (0, 0),
                    "patrol_route": [],
                    "patrol_index": 0,
                    "patrol_complete": True,  # Saltarse patrullaje inicial para pruebas
                    "movement_speed": 3.0,
                    "patrol_initialized": True  # Ya inicializado
                }
                self.patrol_complete = True  # Empezar directamente en procesamiento
            else:
                self.get_logger().warn('⚠️ No se pudo crear workflow de LangGraph')
                self.enable_langgraph = False
                
        except Exception as e:
            self.get_logger().error(f'❌ Error inicializando LangGraph: {e}')
            import traceback
            traceback.print_exc()
            self.workflow = None
            self.enable_langgraph = False
    
    def camera_callback(self, msg: Image):
        """
        Callback mejorado que almacena la imagen para el workflow.
        Además, graba el video cuando el dron alcanza la altura óptima.
        """
        super().camera_callback(msg)
        
        # Almacenar metadata del UAV cuando se captura el frame
        # Esto es crítico para conversión precisa de coordenadas
        self.current_frame_metadata = {
            "uav_x": float(self.position[0]),
            "uav_y": float(self.position[1]),
            "uav_z": float(self.position[2]),
            "uav_yaw": float(self.yaw)
        }
        
        # Almacenar imagen para el workflow de LangGraph
        self.langgraph_state["latest_camera_image"] = msg
        self.langgraph_state["frame_metadata"] = self.current_frame_metadata.copy()
        self.frame_counter += 1
        
        # Si estamos grabando video, agregar frame
        if self.recording_video:
            try:
                # Limitar captura a 2 fps (camera runs at 10 fps)
                # Skip 4 out of every 5 frames (10 fps / 2 fps = 5)
                self.frame_skip_counter += 1
                if self.frame_skip_counter >= 5:
                    self.frame_skip_counter = 0
                    
                    cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "rgb8")
                    # Almacenar metadata junto con el frame
                    frame_data = {
                        "image": cv_image.copy(),
                        "metadata": self.current_frame_metadata.copy()
                    }
                    self.video_frames.append(frame_data)
                
                # Verificar si ya completamos la duración del video
                elapsed = time.time() - self.video_start_time
                if elapsed >= self.video_duration:
                    self._stop_video_recording()
            except Exception as e:
                self.get_logger().error(f'❌ Error capturando frame: {e}')
    
    def control_loop(self):
        """
        Loop de control que hereda del padre y añade detección para iniciar grabación.
        """
        # Verificar si llegamos al primer waypoint ANTES de que el padre lo detecte
        if not self.waypoint_reached_triggered and len(self.waypoints) > 0:
            target = self.waypoints[0]
            distance = np.linalg.norm(self.position - np.array(target[:3]))
            if distance < self.waypoint_tolerance:  # Llegamos al waypoint
                self.get_logger().info('🎥 Iniciando grabación de video en altura óptima')
                self._start_video_recording()
                self.waypoint_reached_triggered = True
        
        # Ejecutar loop de control padre
        super().control_loop()
    
    def patrol_loop(self):
        """
        Loop de patrullaje que ejecuta el workflow de LangGraph.
        Se ejecuta solo si patrol_complete es False.
        """
        if not self.workflow or self.patrol_complete:
            return
        
        try:
            # Actualizar estado con posición actual
            self.langgraph_state["uav_position"] = (
                float(self.position[0]),
                float(self.position[1])
            )
            
            # Ejecutar un paso del workflow
            # Como LangGraph no tiene ejecución por pasos directa,
            # adaptamos el flujo
            if not self.langgraph_state.get("patrol_initialized", False):
                self.get_logger().info('🛸 Iniciando patrullaje...')
                # Simular que ya inicializamos patrullaje
                self._simulate_patrol_initialization()
                self.langgraph_state["patrol_initialized"] = True
            
            # Ejecutar controlador de patrullaje
            if not self.patrol_complete:
                self._execute_patrol_step()
                
        except Exception as e:
            self.get_logger().error(f'❌ Error en patrullaje: {e}')
    
    def _simulate_patrol_initialization(self):
        """
        Simula la inicialización del patrullaje adaptado para ROS 2.
        """
        # Generar ruta de patrullaje circular adaptada al mundo de MicroSim
        world_size = 50.0
        margin = 5.0
        num_points = 8
        
        patrol_route = []
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = margin * math.cos(angle) + self.position[0]
            y = margin * math.sin(angle) + self.position[1]
            patrol_route.append((x, y))
        
        self.langgraph_state["patrol_route"] = patrol_route
        self.langgraph_state["patrol_index"] = 0
        self.langgraph_state["patrol_complete"] = False
        
        self.get_logger().info(f'🗺️ Ruta de patrullaje generada: {len(patrol_route)} puntos')
    
    def _execute_patrol_step(self):
        """
        Ejecuta un paso del patrullaje adaptado para ROS 2.
        """
        patrol_route = self.langgraph_state.get("patrol_route", [])
        patrol_index = self.langgraph_state.get("patrol_index", 0)
        
        if patrol_index >= len(patrol_route):
            self.patrol_complete = True
            self.langgraph_state["patrol_complete"] = True
            self.get_logger().info('✅ Patrullaje completado, iniciando procesamiento')
            return
        
        # Obtener punto objetivo actual
        target_point = patrol_route[patrol_index]
        current_pos = (
            float(self.position[0]),
            float(self.position[1])
        )
        
        # Calcular distancia
        dx = target_point[0] - current_pos[0]
        dy = target_point[1] - current_pos[1]
        distance = math.sqrt(dx**2 + dy**2)
        
        # Si llegamos al punto
        if distance < 2.0:  # 2 metros de tolerancia
            self.langgraph_state["patrol_index"] = patrol_index + 1
            self.get_logger().info(f'📍 Punto {patrol_index + 1}/{len(patrol_route)} alcanzado')
        else:
            # Generar comando hacia el punto
            self._generate_patrol_command(target_point)
    
    def _generate_patrol_command(self, target: tuple):
        """
        Genera comando de velocidad hacia el punto de patrullaje.
        """
        current_pos = np.array([self.position[0], self.position[1]])
        target_pos = np.array(target)
        
        # Calcular velocidad hacia el objetivo
        velocity_command = self.simple_decision_logic(
            current_pos,
            np.array([target_pos[0], target_pos[1], self.position[2]]),
            0.0
        )
        
        self.publish_velocity(*velocity_command)
    
    def processing_loop(self):
        """
        Loop de procesamiento que ejecuta el análisis con LangGraph.
        Se ejecuta solo después de que patrol_complete es True Y llegamos al primer waypoint.
        """
        if not self.enable_langgraph or not self.workflow:
            return
        
        if not self.patrol_complete or self.processing_complete:
            return
        
        # Solo ejecutar si ya llegamos al primer waypoint (altura óptima)
        if self.current_waypoint_index == 0:
            # Verificar si estamos cerca del primer waypoint
            if len(self.waypoints) > 0:
                target = self.waypoints[0]
                distance = np.linalg.norm(self.position - np.array(target[:3]))
                if distance > 5.0:  # Aún no llegamos
                    return
                
                # Si estamos grabando, esperar a que termine
                if self.recording_video:
                    return
        
        try:
            # Verificar que el video esté grabado y abierto antes de ejecutar el workflow
            # El workflow original requiere video_cap para leer frames
            if "video_cap" not in self.langgraph_state or self.langgraph_state["video_cap"] is None:
                self.get_logger().info('⏳ Esperando que el video esté grabado y abierto...', throttle_duration_sec=2.0)
                return
            
            # Actualizar counter
            if self.langgraph_state["current_frame"] >= self.langgraph_state["max_frames_to_process"]:
                self.processing_complete = True
                self.get_logger().info('✅ Procesamiento completado')
                return
            
            self.get_logger().info(
                f'🔍 Ejecutando análisis LangGraph con workflow original... frame {self.langgraph_state["current_frame"] + 1}/{self.langgraph_state["max_frames_to_process"]}',
                throttle_duration_sec=2.0
            )
            
            # Adaptar estado para LangGraph (con video_cap ya disponible)
            adapted_state = adapt_uav_state_for_ros2(
                self.langgraph_state,
                camera_image=None  # No necesitamos imagen live si tenemos video_cap
            )
            
            # Verificar que video_cap esté en el estado adaptado
            if adapted_state.get("video_cap") is None:
                self.get_logger().warn('⚠️ video_cap no está disponible en el estado adaptado')
                return
            
            # Ejecutar workflow original completo
            result = self.workflow.invoke(
                adapted_state,
                config={"recursion_limit": 50}  # Suficiente para el workflow lineal
            )
            
            # Extraer resultados
            extracted = extract_ros2_state_from_langgraph(result)
            self.langgraph_state.update(extracted)
            
            self.get_logger().info(f'✅ Frame {self.langgraph_state["current_frame"]} procesado')
            
            # Convertir ruta a waypoints
            if self.langgraph_state.get("planned_routes"):
                self._convert_routes_to_waypoints()
            
            # Publicar misión si está lista
            if self.langgraph_state.get("mission_brief"):
                self._publish_mission_to_ugv()
                        
        except Exception as e:
            self.get_logger().error(f'❌ Error en procesamiento: {e}')
            import traceback
            traceback.print_exc()
    
    def _convert_routes_to_waypoints(self):
        """
        Convierte las rutas planificadas por LangGraph a waypoints de ROS 2.
        """
        routes = self.langgraph_state.get("planned_routes", [])
        if not routes:
            return
        
        route = routes[0]  # Usar primera ruta
        route_points = route.get("puntos_ruta", [])
        
        if not route_points:
            return
        
        waypoints = []
        for point in route_points:
            pixel_x = point.get("x", 0)
            pixel_y = point.get("y", 0)
            
            # Convertir a coordenadas mundo
            world_x, world_y = self.coord_transformer.pixel_to_world(pixel_x, pixel_y)
            world_z = 15.0
            
            waypoints.append([world_x, world_y, world_z, 0.0])
        
        if waypoints:
            self.waypoints = waypoints
            self.current_waypoint_index = 0
            # NO activar la misión aquí - estos waypoints son solo para el UGV
            # self.mission_active = True  # Comentado para evitar que el dron se mueva
            self.get_logger().info(f'📍 {len(waypoints)} waypoints cargados desde LangGraph')
    
    def _convert_mission_brief_to_world_coords(self, mission_brief: str) -> str:
        """
        Convierte todas las coordenadas del mission_brief de píxeles a coordenadas del mundo
        usando proyección inversa precisa basada en la posición/orientación del UAV.
        
        Usa la metadata del frame más reciente o la posición actual del UAV.
        """
        lines = mission_brief.split('\n')
        converted_lines = []
        
        # Obtener metadata del UAV (prioridad: frame_metadata > current_frame_metadata > posición actual)
        metadata = self.langgraph_state.get("frame_metadata")
        if not metadata:
            metadata = self.current_frame_metadata
        
        # Usar siempre el método básico pixel_to_world que mapea directamente:
        # - pixel (0, 0) → mundo (-50, -50)
        # - pixel (256, 256) → mundo (50, 50)
        # Este método es más simple y confiable para vista cenital
        
        for line in lines:
            # Buscar líneas con coordenadas: "Position=(x, y)" o "({x}, {y})"
            if "Position=(" in line:
                # Formato: "Victim 1: Position=(60, 44), State=crítico, Priority=alta"
                pattern = r'Position=\((\d+),\s*(\d+)\)'
                matches = re.findall(pattern, line)
                if matches:
                    for pixel_x, pixel_y in matches:
                        # Convertir de píxeles a mundo usando método básico
                        world_x, world_y = self.coord_transformer.pixel_to_world(int(pixel_x), int(pixel_y))
                        # Reemplazar en la línea
                        line = line.replace(f'Position=({pixel_x}, {pixel_y})', f'Position=({world_x:.2f}, {world_y:.2f})')
            elif re.match(r'\s*\d+\.\s*\(\d+,\s*\d+\)', line):
                # Formato: "  1. (26, 26) - inicio" o "  2. (60, 44) - victima (ID: 1)"
                pattern = r'(\s*\d+\.\s*\()(\d+),\s*(\d+)(\))'
                matches = re.findall(pattern, line)
                if matches:
                    for prefix, pixel_x, pixel_y, suffix in matches:
                        # Convertir de píxeles a mundo usando método básico
                        world_x, world_y = self.coord_transformer.pixel_to_world(int(pixel_x), int(pixel_y))
                        # Reemplazar en la línea
                        line = line.replace(f'{prefix}{pixel_x}, {pixel_y}{suffix}', f'{prefix}{world_x:.2f}, {world_y:.2f}{suffix}')
            
            converted_lines.append(line)
        
        return '\n'.join(converted_lines)
    
    def _publish_mission_to_ugv(self):
        """
        Publica la misión al UGV.
        """
        mission_brief = self.langgraph_state.get("mission_brief")
        if not mission_brief:
            return
        
        try:
            # Convertir todas las coordenadas de píxeles a mundo
            mission_brief_world = self._convert_mission_brief_to_world_coords(mission_brief)
            
            # Crear estructura JSON completa similar a uav_to_ugv_message.json
            message_data = {
                "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                "from": "UAV",
                "to": "UGV",
                "message_type": "MISSION_BRIEFING",
                "mission_brief": mission_brief_world,
                "status": "READY_FOR_UGV",
                "message_id": f"MSG_{int(time.time())}"
            }
            
            # Convertir a JSON string
            msg = String()
            msg.data = json.dumps(message_data, indent=2, ensure_ascii=False)
            
            self.mission_pub.publish(msg)
            self.get_logger().info('📤 Misión publicada al UGV con coordenadas en sistema del mundo')
            
            # Asegurar que la misión esté desactivada para que el dron no se mueva
            self.mission_active = False
            self.get_logger().info('🛑 Misión desactivada - dron en modo hover')
        except Exception as e:
            self.get_logger().error(f'❌ Error publicando misión: {e}')
            import traceback
            traceback.print_exc()
    
    def _start_video_recording(self):
        """
        Inicia la grabación de video del camera feed.
        """
        self.recording_video = True
        self.video_frames = []
        self.video_start_time = time.time()
        self.frame_skip_counter = 0  # Reset contador de frames
        self.get_logger().info(f'🎬 Grabación iniciada: {self.video_duration}s @ {self.video_fps}fps')
    
    def _stop_video_recording(self):
        """
        Detiene la grabación y guarda el video.
        """
        if not self.recording_video:
            return
        
        self.recording_video = False
        
        if not self.video_frames:
            self.get_logger().warn('⚠️ No se capturaron frames para el video')
            return
        
        try:
            # Crear video output en MultiAgent (donde está UAV_agent.py)
            # Desde microsim/scripts -> MultiAgent hermano es ../../../../MultiAgent
            script_dir = os.path.dirname(os.path.abspath(__file__))
            multiagent_dir = os.path.join(script_dir, "..", "..", "..", "..", "MultiAgent")
            video_path = os.path.join(multiagent_dir, "uav_simulation.mp4")
            video_path = os.path.abspath(video_path)
            self.get_logger().info(f'📁 Ruta del video: {video_path}')
            
            # Extraer el primer frame - puede ser un diccionario o un array directo
            first_frame = self.video_frames[0]
            if isinstance(first_frame, dict):
                # Si es un diccionario, extraer la imagen
                first_image = first_frame["image"]
                # Almacenar metadata de frames para uso posterior
                self.video_frames_metadata = [f.get("metadata", {}) for f in self.video_frames]
            else:
                # Si es un array directo (compatibilidad con código anterior)
                first_image = first_frame
                self.video_frames_metadata = []
            
            height, width = first_image.shape[:2]
            
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(video_path, fourcc, self.video_fps, (width, height))
            
            # Escribir frames - convertir RGB a BGR para OpenCV VideoWriter
            for frame_data in self.video_frames:
                # Extraer imagen si es diccionario
                if isinstance(frame_data, dict):
                    frame = frame_data["image"]
                else:
                    frame = frame_data
                
                bgr_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                self.video_writer.write(bgr_frame)
            
            self.video_writer.release()
            self.get_logger().info(f'✅ Video guardado: {video_path} ({len(self.video_frames)} frames)')
            
            # Actualizar el estado de LangGraph para apuntar al video y abrir video_cap
            self.langgraph_state["video_path"] = video_path
            
            # Guardar metadata de frames en el estado de LangGraph para uso posterior
            if hasattr(self, 'video_frames_metadata') and self.video_frames_metadata:
                # Usar la metadata del primer frame (o promedio) para conversión de coordenadas
                # Por simplicidad, usamos la metadata del primer frame
                if self.video_frames_metadata[0]:
                    self.langgraph_state["frame_metadata"] = self.video_frames_metadata[0]
                    self.get_logger().info(f'📊 Metadata del UAV guardada: pos=({self.video_frames_metadata[0].get("uav_x", 0):.1f}, {self.video_frames_metadata[0].get("uav_y", 0):.1f}), alt={self.video_frames_metadata[0].get("uav_z", 0):.1f}m')
            
            # Abrir video con OpenCV para que LangGraph lo pueda leer
            video_cap = cv2.VideoCapture(video_path)
            if video_cap.isOpened():
                self.langgraph_state["video_cap"] = video_cap
                fps = video_cap.get(cv2.CAP_PROP_FPS)
                total_frames = int(video_cap.get(cv2.CAP_PROP_FRAME_COUNT))
                self.langgraph_state["total_frames"] = total_frames
                self.langgraph_state["current_frame"] = 0  # Reset para empezar desde el primer frame
                self.get_logger().info(f'📹 Video abierto para análisis: {total_frames} frames @ {fps:.1f}fps')
            else:
                self.get_logger().error('❌ No se pudo abrir el video para análisis')
            
        except Exception as e:
            self.get_logger().error(f'❌ Error guardando video: {e}')
            import traceback
            traceback.print_exc()
        finally:
            self.video_writer = None
            self.video_frames = []
            self.video_frames_metadata = []  # Limpiar metadata


def main(args=None):
    rclpy.init(args=args)
    
    controller = UAVLangGraphController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('🛑 Interrupción del usuario')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

