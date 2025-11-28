#!/usr/bin/env python3
"""
Controlador UGV Inteligente con Integración LangGraph

Este controlador integra el sistema MultiAgent UGV con el simulador ROS 2 MicroSim.
Escucha misiones del UAV, ejecuta rescates y detecta colisiones con sensores ROS 2.

Autor: Integración MultiAgent-MicroSim
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import String
import numpy as np
import math
import json
import time
import sys
import os

# Agregar path al módulo multiagent
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from multiagent.coordinate_transforms import CoordinateTransformer


class UGVAIController(Node):
    """
    Controlador UGV inteligente que:
    - Escucha misiones del UAV vía ROS 2 topics
    - Navega hacia víctimas siguiendo rutas planificadas
    - Detecta colisiones con sensor de rango
    - Ejecuta protocolo de rescate de víctimas
    """
    
    def __init__(self):
        super().__init__('ugv_ai_controller')
        
        # ====================================================================
        # CONFIGURACIÓN
        # ====================================================================
        self.control_rate_hz = 10.0  # Frecuencia de control
        self.waypoint_tolerance = 1.0  # metros - tolerancia para llegar a waypoint
        self.max_linear_speed = 1.0  # m/s - velocidad lineal máxima
        self.max_angular_speed = 0.5  # rad/s - velocidad angular máxima
        
        # Detección de colisiones
        self.min_range = 2.0  # metros - distancia mínima antes de detener
        self.warning_range = 5.0  # metros - distancia de advertencia
        
        # ====================================================================
        # ESTADO DEL AGENTE
        # ====================================================================
        self.position = np.array([0.0, 0.0])  # [x, y] en metros
        self.yaw = 0.0  # radianes
        self.velocity = np.array([0.0, 0.0])  # [vx, vy]
        
        # Misión
        self.mission_received = False
        self.mission_data = None
        self.current_waypoint_index = 0
        self.waypoints = []
        self.victims_rescued = []
        
        # Sensor de rango
        self.range_sensor_data = None
        self.last_range_time = None
        
        # Transformador de coordenadas
        # world_size=100.0 porque el mundo va de -50 a +50 metros (100 metros total)
        self.coord_transformer = CoordinateTransformer(
            image_width=256,
            image_height=256,
            world_size=100.0
        )
        
        # ====================================================================
        # ROS 2 PUBLISHERS
        # ====================================================================
        self.cmd_pub = self.create_publisher(Twist, '/rover/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/ugv/status', 10)
        
        # ====================================================================
        # ROS 2 SUBSCRIBERS
        # ====================================================================
        # Escuchar misiones del UAV
        self.mission_sub = self.create_subscription(
            String, '/uav/mission_brief', self.mission_callback, 10
        )
        
        # Odometría para conocer posición actual
        self.odom_sub = self.create_subscription(
            Odometry, '/rover/odom', self.odometry_callback, 10
        )
        
        # Sensor de rango para detección de colisiones
        self.range_sub = self.create_subscription(
            Range, '/rover/range', self.range_callback, 10
        )
        
        # ====================================================================
        # CONTROL LOOP TIMER
        # ====================================================================
        control_period = 1.0 / self.control_rate_hz
        self.control_timer = self.create_timer(control_period, self.control_loop)
        
        self.get_logger().info('🚗 UGV AI Controller inicializado!')
        self.get_logger().info('   Esperando misión del UAV...')
    
    def mission_callback(self, msg: String):
        """
        Callback cuando llega una nueva misión del UAV.
        """
        try:
            mission_data = json.loads(msg.data)
            
            self.get_logger().info('📨 Misión recibida del UAV')
            
            # Procesar misión
            if mission_data.get("route"):
                self._process_mission(mission_data)
            else:
                self.get_logger().warn('⚠️  Misión sin ruta válida')
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'❌ Error parseando misión: {e}')
        except Exception as e:
            self.get_logger().error(f'❌ Error procesando misión: {e}')
    
    def _process_mission(self, mission_data: dict):
        """
        Procesa la misión y actualiza waypoints.
        """
        route = mission_data.get("route")
        route_points = route.get("puntos_ruta", [])
        
        if not route_points:
            self.get_logger().warn('⚠️  No hay puntos en la ruta')
            return
        
        # Convertir puntos de imagen a coordenadas mundo
        waypoints = []
        for point in route_points:
            pixel_x = point.get("x", 0)
            pixel_y = point.get("y", 0)
            tipo = point.get("tipo", "unknown")
            
            # Convertir coordenadas
            world_x, world_y = self.coord_transformer.pixel_to_world(pixel_x, pixel_y)
            
            waypoints.append({
                'position': np.array([world_x, world_y]),
                'type': tipo,
                'victim_id': point.get("victima_id", None)
            })
        
        if waypoints:
            self.waypoints = waypoints
            self.current_waypoint_index = 0
            self.mission_received = True
            self.mission_data = mission_data
            self.get_logger().info(f'✅ {len(waypoints)} waypoints cargados')
            self.get_logger().info('🚀 Iniciando misión de rescate')
    
    def odometry_callback(self, msg: Odometry):
        """
        Callback de odometría - actualiza posición y orientación.
        """
        # Posición
        self.position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])
        
        # Velocidad
        self.velocity = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y
        ])
        
        # Yaw
        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
    
    def range_callback(self, msg: Range):
        """
        Callback del sensor de rango - para detección de colisiones.
        """
        self.range_sensor_data = msg.range
        self.last_range_time = time.time()
    
    def control_loop(self):
        """
        Loop principal de control a 10 Hz.
        """
        if not self.mission_received or not self.waypoints:
            # No hay misión, detenerse
            self.publish_velocity(0.0, 0.0)
            return
        
        # Verificar si la misión está completa
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('✅ Misión completada!')
            self.publish_velocity(0.0, 0.0)
            return
        
        # Obtener waypoint actual
        current_waypoint = self.waypoints[self.current_waypoint_index]
        target_pos = current_waypoint['position']
        
        # Calcular distancia al objetivo
        distance_to_goal = np.linalg.norm(target_pos - self.position)
        
        # Si llegamos al waypoint
        if distance_to_goal < self.waypoint_tolerance:
            self.get_logger().info(
                f'📍 Waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)} alcanzado'
            )
            
            # Si es una víctima, ejecutar protocolo de rescate
            if current_waypoint['type'] == 'victima':
                self._execute_rescue_protocol(current_waypoint)
            
            # Avanzar al siguiente waypoint
            self.current_waypoint_index += 1
            return
        
        # Detección de colisiones
        if self._check_collision_risk():
            self.get_logger().warn('⚠️  Riesgo de colisión detectado!')
            self.publish_velocity(0.0, 0.0)
            return
        
        # Calcular comando de velocidad hacia el objetivo
        velocity_command = self._calculate_velocity_to_target(target_pos)
        
        # Publicar comando
        self.publish_velocity(velocity_command[0], velocity_command[1])
    
    def _check_collision_risk(self) -> bool:
        """
        Verifica riesgo de colisión usando el sensor de rango.
        
        Returns:
            True si hay riesgo de colisión
        """
        if self.range_sensor_data is None:
            return False
        
        # Verificar si el sensor está actualizado (últimos 0.5 segundos)
        if self.last_range_time is None:
            return False
        
        if time.time() - self.last_range_time > 0.5:
            return False
        
        # Si hay un obstáculo muy cerca, hay riesgo
        if self.range_sensor_data < self.min_range:
            return True
        
        return False
    
    def _calculate_velocity_to_target(self, target: np.ndarray) -> tuple:
        """
        Calcula velocidad lineal y angular hacia el objetivo.
        
        Args:
            target: Posición objetivo [x, y]
            
        Returns:
            Tupla (linear_speed, angular_speed)
        """
        # Vector hacia el objetivo
        direction = target - self.position
        distance = np.linalg.norm(direction)
        
        if distance < 0.01:
            return (0.0, 0.0)
        
        # Normalizar dirección
        direction_norm = direction / distance
        
        # Ángulo deseado hacia el objetivo
        target_angle = math.atan2(direction_norm[1], direction_norm[0])
        
        # Error angular
        angle_error = self._normalize_angle(target_angle - self.yaw)
        
        # Control proporcional para velocidad angular
        kp_angular = 1.5
        angular_speed = kp_angular * angle_error
        
        # Limitar velocidad angular
        angular_speed = np.clip(angular_speed, -self.max_angular_speed, self.max_angular_speed)
        
        # Velocidad lineal (reducir si hay que girar mucho)
        linear_speed = self.max_linear_speed * (1.0 - abs(angle_error) / math.pi)
        linear_speed = max(0.0, linear_speed)
        
        return (linear_speed, angular_speed)
    
    def _normalize_angle(self, angle: float) -> float:
        """
        Normaliza un ángulo al rango [-pi, pi].
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def _execute_rescue_protocol(self, waypoint: dict):
        """
        Ejecuta el protocolo de rescate de víctimas.
        """
        victim_id = waypoint.get('victim_id')
        position = waypoint['position']
        
        self.get_logger().info(
            f'🚑 Iniciando protocolo de rescate para víctima {victim_id}'
        )
        self.get_logger().info(
            f'   Ubicación: ({position[0]:.2f}, {position[1]:.2f})'
        )
        
        # Simulación del protocolo
        time.sleep(2.0)  # Evaluación inicial
        time.sleep(1.5)  # Estabilización
        time.sleep(1.0)  # Preparación evacuación
        
        # Registrar víctima rescatada
        self.victims_rescued.append({
            'id': victim_id,
            'position': position.tolist(),
            'timestamp': time.strftime("%Y-%m-%d %H:%M:%S")
        })
        
        self.get_logger().info(
            f'✅ Víctima {victim_id} rescatada! Total: {len(self.victims_rescued)}'
        )
    
    def publish_velocity(self, linear: float, angular: float):
        """
        Publica comando de velocidad.
        
        Args:
            linear: Velocidad lineal en m/s
            angular: Velocidad angular en rad/s
        """
        cmd = Twist()
        cmd.linear.x = float(linear)
        cmd.angular.z = float(angular)
        
        self.cmd_pub.publish(cmd)
    
    def publish_status(self):
        """
        Publica estado del UGV.
        """
        status = {
            'position': self.position.tolist(),
            'mission_active': self.mission_received,
            'waypoint_index': self.current_waypoint_index,
            'victims_rescued': len(self.victims_rescued)
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    controller = UGVAIController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info('🛑 Interrupción del usuario')
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

