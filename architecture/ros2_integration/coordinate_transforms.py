"""
Utilidades para transformación de coordenadas entre imagen y mundo de MicroSim
"""

import numpy as np
import math


class CoordinateTransformer:
    """
    Transforma coordenadas entre diferentes sistemas de referencia:
    - Coordenadas de imagen (píxeles)
    - Coordenadas del mundo de MicroSim (metros)
    
    Mejora: Usa la posición y orientación del UAV para conversión precisa
    usando proyección inversa de cámara pinhole.
    """
    
    def __init__(self, image_width=256, image_height=256, world_size=100.0, fov_deg=90.0):
        """
        Inicializa el transformador.
        
        Args:
            image_width: Ancho de la imagen en píxeles
            image_height: Alto de la imagen en píxeles
            world_size: Tamaño total del mundo de MicroSim en metros (rango total: -world_size/2 a +world_size/2)
                       Por defecto: 100.0 (rango: -50 a +50 metros)
            fov_deg: Campo de visión horizontal de la cámara en grados
        """
        self.image_width = image_width
        self.image_height = image_height
        self.world_size = world_size
        self.fov_deg = fov_deg
        
        # Escalas de conversión (método antiguo, mantenido para compatibilidad)
        self.pixel_to_meter_x = world_size / image_width
        self.pixel_to_meter_y = world_size / image_height
        
        # Parámetros de la cámara pinhole (matching camera.py)
        self.fx = image_width / (2.0 * np.tan(np.radians(fov_deg) / 2.0))
        self.fy = self.fx  # Square pixels
        self.cx = image_width / 2.0
        self.cy = image_height / 2.0
        
        # Ángulo de inclinación de la cámara (pitch down)
        # La cámara está inclinada 15 grados hacia abajo según tf_broadcaster.py
        self.camera_pitch = np.radians(15.0)
    
    def pixel_to_world(self, pixel_x, pixel_y):
        """
        Convierte coordenadas de píxel a coordenadas del mundo.
        
        Mapeo correcto:
        - Esquina superior izquierda: imagen (0, 0) = mundo (-50, -50)
        - Esquina inferior derecha: imagen (256, 256) = mundo (50, 50)
        - Centro: imagen (128, 128) = mundo (0, 0)
        
        Args:
            pixel_x: Coordenada X en píxeles (0 = izquierda)
            pixel_y: Coordenada Y en píxeles (0 = arriba)
            
        Returns:
            Tuple (world_x, world_y) en metros
        """
        # El mundo va de -50 a +50 en ambos ejes (100 metros total)
        world_min = -self.world_size / 2.0  # -50
        world_max = self.world_size / 2.0   # +50
        
        # Normalizar píxel a [0, 1]
        norm_x = pixel_x / self.image_width
        norm_y = pixel_y / self.image_height
        
        # Mapear a coordenadas del mundo
        # X: pixel 0 → -50, pixel 256 → +50 (lineal)
        world_x = world_min + norm_x * (world_max - world_min)
        # Y: Según especificación del usuario:
        #    imagen (0, 0) → mundo (-50, -50)
        #    imagen (256, 256) → mundo (50, 50)
        #    Por lo tanto, Y también es lineal (sin inversión adicional)
        #    pixel_y=0 → world_y=-50, pixel_y=256 → world_y=50
        world_y = world_min + norm_y * (world_max - world_min)
        
        return (world_x, world_y)
    
    def world_to_pixel(self, world_x, world_y):
        """
        Convierte coordenadas del mundo a coordenadas de píxel.
        
        Mapeo inverso:
        - Mundo (-50, -50) → imagen (0, 0)
        - Mundo (50, 50) → imagen (256, 256)
        - Mundo (0, 0) → imagen (128, 128)
        
        Args:
            world_x: Coordenada X en metros
            world_y: Coordenada Y en metros
            
        Returns:
            Tuple (pixel_x, pixel_y) en píxeles
        """
        # El mundo va de -50 a +50 en ambos ejes
        world_min = -self.world_size / 2.0  # -50
        world_max = self.world_size / 2.0   # +50
        
        # Normalizar coordenadas del mundo a [0, 1]
        norm_x = (world_x - world_min) / (world_max - world_min)
        # Para Y, según especificación del usuario:
        #    mundo (-50, -50) → imagen (0, 0)
        #    mundo (50, 50) → imagen (256, 256)
        #    Por lo tanto, Y también es lineal (sin inversión adicional)
        norm_y = (world_y - world_min) / (world_max - world_min)
        
        # Convertir a píxeles
        pixel_x = norm_x * self.image_width
        pixel_y = norm_y * self.image_height
        
        # Asegurar que estén dentro de los límites
        pixel_x = max(0, min(self.image_width - 1, pixel_x))
        pixel_y = max(0, min(self.image_height - 1, pixel_y))
        
        return (int(pixel_x), int(pixel_y))
    
    def pixel_to_world_with_altitude(self, pixel_x, pixel_y, drone_altitude):
        """
        Convierte coordenadas de píxel a coordenadas del mundo 3D.
        
        Args:
            pixel_x: Coordenada X en píxeles
            pixel_y: Coordenada Y en píxeles
            drone_altitude: Altura del drone en metros
        
        Returns:
            Tuple (world_x, world_y, world_z) en metros
        """
        world_x, world_y = self.pixel_to_world(pixel_x, pixel_y)
        
        # Para simplicidad, asumimos que el suelo está en z=0
        # Esto puede ajustarse según el escenario de MicroSim
        world_z = 0.0
        
        return (world_x, world_y, world_z)
    
    def pixel_to_world_precise(self, pixel_x, pixel_y, 
                               drone_x, drone_y, drone_z, drone_yaw):
        """
        Convierte coordenadas de píxel a coordenadas del mundo usando
        proyección inversa de cámara pinhole con la posición real del UAV.
        
        Este método es más preciso que pixel_to_world() porque considera:
        - La posición real del UAV (x, y, z)
        - La orientación del UAV (yaw)
        - La geometría real de la cámara (FOV, proyección pinhole)
        - La inclinación de la cámara (pitch down)
        
        Args:
            pixel_x: Coordenada X en píxeles (0 = izquierda)
            pixel_y: Coordenada Y en píxeles (0 = arriba)
            drone_x: Posición X del UAV en metros (coordenadas del mundo)
            drone_y: Posición Y del UAV en metros (coordenadas del mundo)
            drone_z: Altura del UAV en metros (sobre el suelo)
            drone_yaw: Orientación del UAV en radianes (0 = norte, positivo = rotación horaria)
        
        Returns:
            Tuple (world_x, world_y) en metros (posición en el suelo)
        """
        # 1. Convertir píxel a coordenadas normalizadas de la cámara
        # Sistema de coordenadas de la cámara: X=right, Y=down, Z=forward
        x_cam = (pixel_x - self.cx) / self.fx
        y_cam = (pixel_y - self.cy) / self.fy
        z_cam = -1.0  # Cámara apunta hacia abajo (Z negativo)
        
        # Normalizar dirección del rayo
        ray_length = math.sqrt(x_cam**2 + y_cam**2 + z_cam**2)
        x_cam /= ray_length
        y_cam /= ray_length
        z_cam /= ray_length
        
        # 2. Aplicar rotación de pitch (cámara inclinada hacia abajo)
        cos_pitch = math.cos(self.camera_pitch)
        sin_pitch = math.sin(self.camera_pitch)
        
        x_pitch = x_cam
        y_pitch = y_cam * cos_pitch - z_cam * sin_pitch
        z_pitch = y_cam * sin_pitch + z_cam * cos_pitch
        
        # 3. Aplicar rotación de yaw (orientación del UAV)
        # En el mundo: +X = este, +Y = norte
        # Yaw: 0 = norte (+Y), positivo = rotación horaria
        cos_yaw = math.cos(drone_yaw)
        sin_yaw = math.sin(drone_yaw)
        
        x_world = x_pitch * cos_yaw - y_pitch * sin_yaw
        y_world = x_pitch * sin_yaw + y_pitch * cos_yaw
        z_world = z_pitch
        
        # 4. Calcular intersección del rayo con el plano del suelo (z = 0)
        # Rayo: P(t) = drone_pos + t * ray_direction
        # Suelo: z = 0
        # drone_z + t * z_world = 0
        # t = -drone_z / z_world
        
        if abs(z_world) < 1e-6:
            # Rayo casi paralelo al suelo, usar método aproximado
            # Proyectar usando altura y distancia angular
            ground_distance = drone_z * math.sqrt(x_world**2 + y_world**2) / abs(z_world) if abs(z_world) > 1e-6 else 0
            world_x = drone_x + x_world * ground_distance
            world_y = drone_y + y_world * ground_distance
        else:
            t = -drone_z / z_world
            world_x = drone_x + t * x_world
            world_y = drone_y + t * y_world
        
        return (world_x, world_y)

