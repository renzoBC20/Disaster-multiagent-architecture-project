#!/usr/bin/env python3
"""
Controlador UGV con Integración Completa de LangGraph

Este controlador integra el workflow completo de StateGraph de LangGraph
del sistema MultiAgent con ROS 2, permitiendo que el rover reciba misiones
del UAV y las ejecute de forma autónoma.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import numpy as np
import math
import json
import time
import sys
import os

# Agregar path al módulo ros2_integration
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'ros2_integration'))

from langgraph_workflow import (
    create_ugv_workflow_for_ros2,
    adapt_ugv_state_for_ros2,
    extract_ugv_ros2_state_from_langgraph
)


class UGVLangGraphController(Node):
    """
    Controlador UGV que usa el workflow completo de LangGraph.
    
    Recibe misiones del UAV y las ejecuta usando el workflow de LangGraph.
    """
    
    def __init__(self):
        super().__init__('ugv_langgraph_controller')
        
        # ====================================================================
        # CONFIGURACIÓN
        # ====================================================================
        self.workflow = None
        self.langgraph_state = {}  # Estado compartido de LangGraph
        
        # ====================================================================
        # ESTADO DEL AGENTE
        # ====================================================================
        self.mission_received = False
        self.mission_data = {}
        self.current_position = np.array([0.0, 0.0])
        self.current_orientation = 0.0  # Orientación actual del rover (radianes)
        self.target_point = None
        self.route_index = 0
        self.arrival_confirmed = False
        self.mission_status = "WAITING"
        self.movement_speed = 2.0  # m/s
        self.waypoint_tolerance = 0.5  # metros
        
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
        
        # ====================================================================
        # ROS 2 PUBLISHERS
        # ====================================================================
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/rover/cmd_vel', 10
        )
        
        # ====================================================================
        # INICIALIZAR LANGGRAPH WORKFLOW
        # ====================================================================
        self._initialize_langgraph_workflow()
        
        # ====================================================================
        # TIMERS
        # ====================================================================
        # Timer para ejecutar el workflow de LangGraph
        self.workflow_timer = self.create_timer(2.0, self.workflow_loop)
        
        # Timer para control de movimiento
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('🚗 UGV LangGraph Controller inicializado!')
        self.get_logger().info('   Esperando mision del UAV...')
    
    def _initialize_langgraph_workflow(self):
        """
        Inicializa el workflow de LangGraph.
        """
        try:
            self.workflow = create_ugv_workflow_for_ros2()
            if self.workflow:
                self.get_logger().info('✅ Workflow de LangGraph inicializado')
            else:
                self.get_logger().error('❌ No se pudo inicializar el workflow de LangGraph')
        except Exception as e:
            self.get_logger().error(f'❌ Error inicializando workflow: {e}')
            import traceback
            traceback.print_exc()
    
    def mission_callback(self, msg: String):
        """
        Callback cuando llega una nueva misión del UAV.
        """
        try:
            mission_json = json.loads(msg.data)
            mission_brief = mission_json.get("mission_brief", "")
            
            self.get_logger().info('📨 Mision recibida del UAV')
            
            # Parsear el mission brief
            self.mission_data = self._parse_mission_brief(mission_brief)
            
            if self.mission_data and self.mission_data.get("route"):
                self.mission_received = True
                self.get_logger().info(f'✅ Mision parseada: {len(self.mission_data.get("victims", []))} victimas, {len(self.mission_data.get("obstacles", []))} obstaculos, {len(self.mission_data.get("route", {}).get("points", []))} puntos de ruta')
                
                # Actualizar estado de LangGraph
                self.langgraph_state["mission_received"] = True
                self.langgraph_state["mission_data"] = self.mission_data
            else:
                self.get_logger().warn('⚠️  Mision sin datos validos')
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'❌ Error parseando JSON de mision: {e}')
        except Exception as e:
            self.get_logger().error(f'❌ Error procesando mision: {e}')
            import traceback
            traceback.print_exc()
    
    def odometry_callback(self, msg: Odometry):
        """
        Callback para actualizar la posición actual del rover.
        """
        position = msg.pose.pose.position
        self.current_position = np.array([position.x, position.y])
        
        # Extraer orientación del quaternion
        orientation = msg.pose.pose.orientation
        # Convertir quaternion a yaw (ángulo alrededor del eje Z)
        # Para un quaternion (x, y, z, w), el yaw se calcula como:
        # yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
        w = orientation.w
        x = orientation.x
        y = orientation.y
        z = orientation.z
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        self.current_orientation = math.atan2(siny_cosp, cosy_cosp)
    
    def _parse_mission_brief(self, mission_brief: str) -> dict:
        """
        Parsea el mission brief del UAV y extrae víctimas, obstáculos y ruta.
        Las coordenadas ya vienen en sistema del mundo (centro de imagen = 0,0).
        """
        try:
            mission_data = {
                "victims": [],
                "obstacles": [],
                "route": {}
            }

            # Buscar sección de víctimas (puede tener emoji o no)
            if "VÍCTIMAS A RESCATAR" in mission_brief or "VÍCTIMAS A VISITAR" in mission_brief or "VICTIMS A VISITAR" in mission_brief or "VICTIMS" in mission_brief:
                lines = mission_brief.split('\n')
                in_victims_section = False

                for line in lines:
                    if "VÍCTIMAS" in line or "VICTIMS" in line:
                        in_victims_section = True
                        continue
                    if in_victims_section and ("OBSTÁCULOS" in line or "OBSTACULOS" in line or "OPTIMIZED" in line or "ROUTE" in line):
                        break
                    if in_victims_section and "Position=" in line:
                        # Extraer información de víctima: "Victim 1: Position=(40, 440), State=crítico, Priority=alta"
                        parts = line.split("Position=")
                        if len(parts) > 1:
                            # Extraer coordenadas entre paréntesis: "(40, 440)"
                            coords_str = parts[1].strip()
                            if coords_str.startswith("("):
                                coords_end = coords_str.find(")")
                                if coords_end > 0:
                                    coords_part = coords_str[1:coords_end]  # Obtener "40, 440"
                                    coord_values = [v.strip() for v in coords_part.split(",")]
                                    if len(coord_values) >= 2:
                                        x, y = float(coord_values[0]), float(coord_values[1])

                                        # Extraer ID, estado y prioridad
                                        victim_id = len(mission_data["victims"]) + 1
                                        state = "crítico" if "crítico" in line or "critico" in line else "herido" if "herido" in line else "seguro"
                                        priority = "alta" if "alta" in line else "media" if "media" in line else "baja"

                                        # Las coordenadas ya vienen en sistema del mundo (centro de imagen = 0,0)
                                        # No necesitamos convertir
                                        mission_data["victims"].append({
                                            "id": victim_id,
                                            "coordenadas": {"x": x, "y": y},
                                            "estado": state,
                                            "prioridad": priority
                                        })
                                    else:
                                        self.get_logger().warn(f'⚠️ Formato de coordenadas invalido en linea: {line}')
                                        continue
                                else:
                                    self.get_logger().warn(f'⚠️ No se encontro cierre de parentesis en: {line}')
                                    continue
                            else:
                                self.get_logger().warn(f'⚠️ Coordenadas no empiezan con parentesis en: {line}')
                                continue

            # Buscar sección de obstáculos (puede tener emoji o no)
            if "OBSTÁCULOS IDENTIFICADOS" in mission_brief or "OBSTÁCULOS" in mission_brief or "OBSTACULOS" in mission_brief:
                lines = mission_brief.split('\n')
                in_obstacles_section = False

                for line in lines:
                    if "OBSTÁCULOS" in line or "OBSTACULOS" in line:
                        in_obstacles_section = True
                        continue
                    if in_obstacles_section and ("OPTIMIZED" in line or "ROUTE" in line or "Puntos de ruta" in line):
                        break
                    if in_obstacles_section and "Position=" in line:
                        # Similar a víctimas
                        parts = line.split("Position=")
                        if len(parts) > 1:
                            # Extraer coordenadas entre paréntesis: "(40, 440)"
                            coords_str = parts[1].strip()
                            if coords_str.startswith("("):
                                coords_end = coords_str.find(")")
                                if coords_end > 0:
                                    coords_part = coords_str[1:coords_end]  # Obtener "40, 440"
                                    coord_values = [v.strip() for v in coords_part.split(",")]
                                    if len(coord_values) >= 2:
                                        x, y = float(coord_values[0]), float(coord_values[1])

                                        obstacle_id = len(mission_data["obstacles"]) + 1
                                        obstacle_type = "edificio" if "edificio" in line else "escombro" if "escombro" in line else "vehículo"

                                        # Las coordenadas ya vienen en sistema del mundo (centro de imagen = 0,0)
                                        # No necesitamos convertir
                                        mission_data["obstacles"].append({
                                            "id": obstacle_id,
                                            "coordenadas": {"x": x, "y": y},
                                            "tipo": obstacle_type
                                        })
                                    else:
                                        self.get_logger().warn(f'⚠️ Formato de coordenadas invalido en linea: {line}')
                                        continue
                                else:
                                    self.get_logger().warn(f'⚠️ No se encontro cierre de parentesis en: {line}')
                                    continue
                            else:
                                self.get_logger().warn(f'⚠️ Coordenadas no empiezan con parentesis en: {line}')
                                continue

            # Buscar ruta optimizada
            if "Puntos de ruta:" in mission_brief or "Route points:" in mission_brief:
                lines = mission_brief.split('\n')
                in_route_section = False
                route_points = []

                for i, line in enumerate(lines):
                    if "Puntos de ruta:" in line or "Route points:" in line:
                        in_route_section = True
                        continue
                    # Si estamos en la sección de ruta, procesar líneas que contengan coordenadas
                    if not in_route_section:
                        continue

                    # Si la línea está vacía o solo tiene espacios, continuar
                    if not line.strip():
                        continue
                    
                    # Si encontramos una línea que claramente no es parte de la ruta (como comillas o llaves de cierre JSON), detener
                    if line.strip().startswith('"') or line.strip().startswith('}'):
                        break

                    # Buscar coordenadas en la línea
                    if "(" in line and ")" in line:
                        # Extraer punto: "1. (10, 10) - inicio" o "2. (40, 440) - victima (ID: 1)"
                        try:
                            # Buscar coordenadas entre paréntesis
                            coords_start = line.find("(")
                            coords_end = line.find(")", coords_start)
                            if coords_start >= 0 and coords_end > coords_start:
                                coords_part = line[coords_start+1:coords_end].strip()
                                coord_values = [v.strip() for v in coords_part.split(",")]
                                if len(coord_values) >= 2:
                                    x, y = float(coord_values[0]), float(coord_values[1])
                                    
                                    # Determinar tipo
                                    tipo = "inicio" if "inicio" in line else "victima" if "victima" in line else "punto_paso"
                                    victima_id = None
                                    if "victima" in line and "ID:" in line:
                                        id_part = line.split("ID:")[1].split()[0].strip()
                                        # Quitar paréntesis de cierre si existe
                                        id_part = id_part.rstrip(')').strip()
                                        try:
                                            victima_id = int(id_part)
                                        except ValueError:
                                            self.get_logger().warn(f'⚠️ No se pudo parsear ID de victima: {id_part}')
                                            victima_id = None
                                    
                                    # Las coordenadas ya vienen en sistema del mundo (centro de imagen = 0,0)
                                    # No necesitamos convertir
                                    route_points.append({
                                        "coordenadas": {"x": x, "y": y},
                                        "tipo": tipo,
                                        "victima_id": victima_id
                                    })
                                else:
                                    self.get_logger().warn(f'⚠️ Formato de coordenadas invalido en punto de ruta: {line}')
                                    continue
                            else:
                                self.get_logger().warn(f'⚠️ No se encontraron parentesis validos en: {line}')
                                continue
                        except Exception as e:
                            self.get_logger().warn(f'⚠️ Error parseando punto de ruta: {e}')
                
                if route_points:
                    mission_data["route"] = {
                        "points": route_points,
                        "total_points": len(route_points)
                    }
                    self.get_logger().info(f'✅ Ruta parseada: {len(route_points)} puntos encontrados')
                else:
                    self.get_logger().warn(f'⚠️ No se encontraron puntos de ruta en el mission brief')
            
            self.get_logger().info(f'📊 Mision parseada: {len(mission_data.get("victims", []))} victimas, {len(mission_data.get("obstacles", []))} obstaculos, {len(mission_data.get("route", {}).get("points", []))} puntos de ruta')
            return mission_data
            
        except Exception as e:
            self.get_logger().error(f'❌ Error parseando mission brief: {e}')
            import traceback
            traceback.print_exc()
            return {}
    
    def workflow_loop(self):
        """
        Loop que ejecuta el workflow de LangGraph.
        """
        if not self.workflow or not self.mission_received:
            return
        
        # Prevenir ejecución si estamos esperando llegada en modo ROS 2
        if self.mission_status == "WAITING_FOR_ARRIVAL" and not self.arrival_confirmed:
            return
        
        try:
            # Preparar estado ROS 2
            ros2_state = {
                "mission_received": self.mission_received,
                "mission_data": self.mission_data,
                "position": self.current_position,
                "target_point": self.target_point,
                "route_index": self.route_index,
                "arrival_confirmed": self.arrival_confirmed,
                "mission_status": self.mission_status,
                "movement_speed": self.movement_speed
            }
            
            # Adaptar estado para LangGraph
            langgraph_state = adapt_ugv_state_for_ros2(ros2_state)
            
            # Ejecutar workflow
            result = self.workflow.invoke(langgraph_state)
            
            # Extraer estado de vuelta a ROS 2
            extracted = extract_ugv_ros2_state_from_langgraph(result)
            
            # Actualizar estado local
            self.mission_status = extracted.get("mission_status", self.mission_status)
            self.route_index = extracted.get("route_index", self.route_index)
            self.target_point = extracted.get("target_point")
            self.arrival_confirmed = extracted.get("arrival_confirmed", False)
            
            # Log de estado
            self.get_logger().info(f'🔍 Estado despues del workflow: status={extracted.get("mission_status")}, route_index={extracted.get("route_index")}, arrival_confirmed={extracted.get("arrival_confirmed")}')
            
        except Exception as e:
            self.get_logger().error(f'❌ Error ejecutando workflow: {e}')
            import traceback
            traceback.print_exc()
    
    def control_loop(self):
        """
        Loop de control que publica comandos de velocidad hacia el punto objetivo.
        """
        # Permitir movimiento si estamos en estado MOVING o WAITING_FOR_ARRIVAL (moviéndonos hacia el objetivo)
        if (self.mission_status not in ["MOVING", "WAITING_FOR_ARRIVAL"]) or self.target_point is None:
            # Publicar velocidad cero si no hay objetivo
            self.publish_velocity(0.0, 0.0, 0.0)
            return
        
        # Calcular distancia al objetivo
        dx = self.target_point[0] - self.current_position[0]
        dy = self.target_point[1] - self.current_position[1]
        distance = math.sqrt(dx**2 + dy**2)
        
        # Verificar si hemos llegado al objetivo
        if distance < self.waypoint_tolerance:
            if not self.arrival_confirmed:
                self.get_logger().info(f'🎯 Waypoint alcanzado: ({self.target_point[0]:.2f}, {self.target_point[1]:.2f})')
                self.arrival_confirmed = True
                self.publish_velocity(0.0, 0.0, 0.0)
            return
        
        # Calcular velocidades
        linear_vel, angular_vel = self._calculate_cmd_vel(dx, dy, distance)
        self.publish_velocity(linear_vel, 0.0, angular_vel)
    
    def _calculate_cmd_vel(self, dx: float, dy: float, distance: float) -> tuple:
        """
        Calcula las velocidades lineal y angular necesarias para moverse hacia el objetivo.
        Usa control proporcional mejorado con límites de velocidad.
        
        Args:
            dx: Diferencia en X
            dy: Diferencia en Y
            distance: Distancia total al objetivo
        
        Returns:
            Tuple (linear_vel, angular_vel)
        """
        # Ángulo hacia el objetivo
        target_angle = math.atan2(dy, dx)
        
        # Calcular diferencia de ángulo (normalizada a [-pi, pi])
        angle_error = target_angle - self.current_orientation
        # Normalizar a [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        
        # Parámetros mejorados del controlador
        max_linear_vel = self.movement_speed  # Velocidad máxima lineal
        max_angular_vel = 1.5  # Velocidad máxima angular (rad/s)
        angle_tolerance = 0.15  # radianes (~8.5 grados) - más estricto
        
        # Si el ángulo de error es grande, girar primero antes de avanzar
        if abs(angle_error) > angle_tolerance:
            # Girar en su lugar con control proporcional
            kp_angular = 1.2  # Ganancia angular
            angular_vel = kp_angular * angle_error
            # Limitar velocidad angular
            angular_vel = max(-max_angular_vel, min(max_angular_vel, angular_vel))
            linear_vel = 0.0
        else:
            # Avanzar y ajustar dirección simultáneamente
            # Control proporcional para velocidad lineal (más cerca = más lento)
            kp_linear = 0.8  # Ganancia lineal
            linear_vel = min(max_linear_vel, kp_linear * distance)
            # Reducir velocidad lineal si hay error angular significativo
            if abs(angle_error) > 0.05:
                linear_vel *= (1.0 - abs(angle_error) / (math.pi / 2))
            
            # Control proporcional para velocidad angular
            kp_angular = 0.8  # Ganancia angular más suave
            angular_vel = kp_angular * angle_error
            # Limitar velocidad angular
            angular_vel = max(-max_angular_vel, min(max_angular_vel, angular_vel))
        
        return linear_vel, angular_vel
    
    def publish_velocity(self, linear_x: float, linear_y: float, angular_z: float):
        """
        Publica un comando de velocidad.
        """
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    controller = UGVLangGraphController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
