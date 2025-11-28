#!/usr/bin/env python3
"""
Controlador UAV Inteligente con Integración LangGraph

Este controlador integra el sistema MultiAgent (LangGraph + OpenAI) 
con el simulador ROS 2 MicroSim para proporcionar control inteligente
del drone basado en análisis de visión y planificación con IA.

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
import sys
import os
import time
import json

# Agregar path al módulo multiagent
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from multiagent.langgraph_integration import (
    encode_frame_to_base64,
    identify_victims_from_image,
    identify_obstacles_from_image,
    plan_route,
    convert_image_to_cv2
)
from multiagent.coordinate_transforms import CoordinateTransformer
from scripts.autonomous_drone_controller import AutonomousDroneController


class UAVAIController(AutonomousDroneController):
    """
    Controlador UAV inteligente que usa análisis de visión y planificación con IA.
    
    Extiende el controlador básico con:
    - Análisis de víctimas y obstáculos usando GPT-4o Vision
    - Planificación de rutas inteligente
    - Comunicación con UGV vía ROS 2 topics
    - Coordenadas transformadas entre imagen y mundo
    """
    
    def __init__(self):
        super().__init__()
        
        # ====================================================================
        # CONFIGURACIÓN DE IA
        # ====================================================================
        self.enable_ai_analysis = True  # Activar análisis con IA
        self.analysis_frequency = 5  # Analizar cada N frames (para no sobrecargar)
        self.frame_counter = 0
        
        # ====================================================================
        # ESTADO DEL AGENTE
        # ====================================================================
        self.detected_victims = []  # Víctimas detectadas
        self.detected_obstacles = []  # Obstáculos detectados
        self.planned_route = None  # Ruta planificada por IA
        self.mission_brief = None  # Briefing para UGV
        
        # Transformador de coordenadas
        # world_size=100.0 porque el mundo va de -50 a +50 metros (100 metros total)
        self.coord_transformer = CoordinateTransformer(
            image_width=256,  # Ajustar según resolución de MicroSim
            image_height=256,
            world_size=100.0  # Tamaño total del mundo en metros (rango: -50 a +50)
        )
        
        # ====================================================================
        # ROS 2 PUBLISHERS PARA COMUNICACIÓN CON UGV
        # ====================================================================
        self.mission_pub = self.create_publisher(
            String, '/uav/mission_brief', 10
        )
        self.status_pub = self.create_publisher(
            String, '/uav/status', 10
        )
        
        # ====================================================================
        # TIMER PARA ANÁLISIS PERIÓDICO
        # ====================================================================
        # Análisis asíncrono cada 2 segundos (para no bloquear control loop)
        self.analysis_timer = self.create_timer(2.0, self.ai_analysis_callback)
        
        self.get_logger().info('🚁 UAV AI Controller inicializado!')
        self.get_logger().info(f'   IA activada: {self.enable_ai_analysis}')
        self.get_logger().info('   Listo para análisis inteligente')
    
    def camera_callback(self, msg: Image):
        """
        Callback mejorado que procesa imágenes con IA.
        """
        # Actualizar imagen más reciente
        super().camera_callback(msg)
        
        # Contador de frames para controlar frecuencia de análisis
        self.frame_counter += 1
        
        # El análisis real se hace en el timer separado para no bloquear
        # Aquí solo almacenamos la imagen
    
    def ai_analysis_callback(self):
        """
        Callback periódico para análisis de IA.
        Se ejecuta en un timer separado para no bloquear el control loop.
        """
        if not self.enable_ai_analysis:
            return
        
        if self.latest_camera_image is None:
            return
        
        # Analizar solo cada N frames para no sobrecargar
        if self.frame_counter % self.analysis_frequency != 0:
            return
        
        try:
            self.get_logger().info('🔍 Iniciando análisis de IA...', throttle_duration_sec=5.0)
            
            # Convertir imagen ROS 2 → OpenCV
            cv_image = convert_image_to_cv2(self.latest_camera_image)
            if cv_image is None:
                return
            
            # Codificar para GPT
            frame_base64 = encode_frame_to_base64(cv_image, self.frame_counter)
            if not frame_base64:
                return
            
            # Identificar víctimas
            self.get_logger().info('👥 Identificando víctimas...')
            victims_result = identify_victims_from_image(
                cv_image, 
                frame_base64,
                current_position=(float(self.position[0]), float(self.position[1]), float(self.position[2]))
            )
            
            if victims_result.get("victimas_identificadas"):
                self.detected_victims = victims_result["victimas_identificadas"]
                self.get_logger().info(
                    f'✅ Detectadas {len(self.detected_victims)} víctimas'
                )
            
            # Identificar obstáculos
            self.get_logger().info('🚧 Identificando obstáculos...')
            obstacles_result = identify_obstacles_from_image(cv_image, frame_base64)
            
            if obstacles_result.get("obstaculos_identificados"):
                self.detected_obstacles = obstacles_result["obstaculos_identificados"]
                self.get_logger().info(
                    f'✅ Detectados {len(self.detected_obstacles)} obstáculos'
                )
            
            # Si tenemos víctimas, planificar ruta
            if self.detected_victims:
                self.get_logger().info('🗺️ Planificando ruta...')
                route_result = plan_route(
                    self.detected_victims,
                    self.detected_obstacles,
                    start_position=(
                        float(self.position[0]),
                        float(self.position[1]),
                        float(self.position[2])
                    )
                )
                
                if route_result.get("ruta_optimizada"):
                    self.planned_route = route_result["ruta_optimizada"]
                    self._convert_route_to_waypoints()
                    self.get_logger().info('✅ Ruta planificada y actualizada')
                
                # Generar mission brief para UGV
                self._generate_mission_brief()
                
                # Publicar misión al UGV
                self._publish_mission_to_ugv()
            
        except Exception as e:
            self.get_logger().error(f'❌ Error en análisis de IA: {e}')
    
    def _convert_route_to_waypoints(self):
        """
        Convierte la ruta planificada por IA a waypoints del controlador.
        """
        if not self.planned_route:
            return
        
        waypoints = []
        route_points = self.planned_route.get("puntos_ruta", [])
        
        for point in route_points:
            pixel_x = point.get("x", 0)
            pixel_y = point.get("y", 0)
            
            # Convertir de coordenadas de imagen a coordenadas mundo
            world_x, world_y = self.coord_transformer.pixel_to_world(pixel_x, pixel_y)
            
            # Altitud fija (ajustable)
            world_z = 15.0  # metros
            yaw = 0.0  # Orientación (ajustable)
            
            waypoints.append([world_x, world_y, world_z, yaw])
        
        if waypoints:
            self.waypoints = waypoints
            self.current_waypoint_index = 0
            self.get_logger().info(f'📍 {len(waypoints)} waypoints cargados desde ruta IA')
    
    def _generate_mission_brief(self):
        """
        Genera el briefing de misión para el UGV.
        """
        if not self.detected_victims:
            return
        
        brief = {
            "mission_id": f"mission_{int(time.time())}",
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "priority": "HIGH",
            "victims": self.detected_victims,
            "obstacles": self.detected_obstacles,
            "route": self.planned_route
        }
        
        self.mission_brief = brief
    
    def _publish_mission_to_ugv(self):
        """
        Publica la misión al UGV vía ROS 2 topic.
        """
        if not self.mission_brief:
            return
        
        try:
            mission_json = json.dumps(self.mission_brief, indent=2)
            
            msg = String()
            msg.data = mission_json
            
            self.mission_pub.publish(msg)
            self.get_logger().info('📤 Misión publicada al UGV')
            
        except Exception as e:
            self.get_logger().error(f'❌ Error publicando misión: {e}')
    
    def simple_decision_logic(self, current_pos, target_pos, target_yaw):
        """
        Lógica de decisión mejorada que puede usar información de IA.
        
        Por ahora, usa la lógica básica pero preparada para
        incorporar decisiones del agente LangGraph.
        """
        # Llamar a la lógica básica del padre
        return super().simple_decision_logic(current_pos, target_pos, target_yaw)
    
    def start_mission(self):
        """
        Inicia la misión autónoma.
        """
        if self.planned_route and self.waypoints:
            self.mission_active = True
            self.get_logger().info('🚀 Misión iniciada con ruta planificada por IA')
        else:
            # Si no hay ruta IA, usar waypoints por defecto
            super().start_mission()


def main(args=None):
    rclpy.init(args=args)
    
    controller = UAVAIController()
    
    try:
        # Iniciar misión después de un breve delay para permitir análisis inicial
        import time
        time.sleep(3.0)  # Esperar a que lleguen algunas imágenes
        
        if controller.enable_ai_analysis:
            controller.get_logger().info('⏳ Esperando análisis inicial de IA...')
            time.sleep(5.0)  # Dar tiempo para primer análisis
        
        controller.start_mission()
        
        rclpy.spin(controller)
        
    except KeyboardInterrupt:
        controller.get_logger().info('🛑 Interrupción del usuario')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

