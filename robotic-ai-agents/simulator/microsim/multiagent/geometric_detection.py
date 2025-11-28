"""
Detección geométrica de víctimas y obstáculos usando procesamiento de imagen.
Combina detección geométrica (Hough/contornos) con clasificación GPT para precisión.
"""

import cv2
import numpy as np
from typing import List, Dict, Tuple, Optional
import math


class GeometricDetector:
    """
    Detecta víctimas (círculos) y obstáculos (cuadrados) usando procesamiento de imagen.
    """
    
    def __init__(self, image_width: int = 256, image_height: int = 256):
        """
        Inicializa el detector geométrico.
        
        Args:
            image_width: Ancho de la imagen
            image_height: Alto de la imagen
        """
        self.image_width = image_width
        self.image_height = image_height
        
        # Parámetros para detección de círculos (HoughCircles)
        self.circle_params = {
            'dp': 1,              # Resolución del acumulador inverso
            'minDist': 20,        # Distancia mínima entre círculos
            'param1': 50,         # Umbral superior para detección de bordes
            'param2': 30,         # Umbral para detección de centros
            'minRadius': 3,       # Radio mínimo del círculo
            'maxRadius': 15       # Radio máximo del círculo
        }
        
        # Umbrales de color para identificar diferentes objetos
        # En formato HSV aproximado (necesitará ajuste según la paleta real)
        self.color_ranges = {
            'red': [(0, 100, 100), (10, 255, 255)],      # Rojo
            'orange': [(10, 100, 100), (25, 255, 255)],  # Naranja (#FF6700)
            'green_dark': [(40, 100, 50), (80, 255, 200)], # Verde oscuro
            'brown': [(10, 50, 50), (20, 255, 200)],     # Marrón (edificios)
            'magenta': [(150, 50, 50), (170, 255, 255)], # Magenta (escombros)
            'blue': [(100, 50, 50), (130, 255, 255)],    # Azul (rover)
            'gray': [(0, 0, 50), (180, 50, 200)],        # Gris (vehículos)
            'yellow': [(20, 100, 100), (30, 255, 255)]   # Amarillo (árboles)
        }
    
    def detect_circles(self, image: np.ndarray) -> List[Dict]:
        """
        Detecta círculos en la imagen usando HoughCircles.
        
        Args:
            image: Imagen RGB (numpy array)
        
        Returns:
            Lista de círculos detectados con su centro y radio
            [{"x": int, "y": int, "radius": int, "bbox": (x, y, w, h)}, ...]
        """
        # Convertir a escala de grises
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        
        # Aplicar desenfoque para reducir ruido
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Detectar círculos
        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            **self.circle_params
        )
        
        candidates = []
        if circles is not None:
            circles = np.uint16(np.around(circles))
            for circle in circles[0, :]:
                x, y, r = int(circle[0]), int(circle[1]), int(circle[2])
                
                # Filtrar círculos que están muy cerca de los bordes
                margin = 5
                if (x - r < margin or x + r > self.image_width - margin or
                    y - r < margin or y + r > self.image_height - margin):
                    continue
                
                # Calcular bounding box
                bbox = (max(0, x - r), max(0, y - r), 
                        min(self.image_width, x + r) - max(0, x - r),
                        min(self.image_height, y + r) - max(0, y - r))
                
                candidates.append({
                    "x": x,
                    "y": y,
                    "radius": r,
                    "bbox": bbox,
                    "type": "circle"
                })
        
        return candidates
    
    def detect_squares(self, image: np.ndarray) -> List[Dict]:
        """
        Detecta cuadrados/rectángulos en la imagen usando contornos.
        
        Args:
            image: Imagen RGB (numpy array)
        
        Returns:
            Lista de cuadrados detectados
            [{"x": int, "y": int, "width": int, "height": int, "bbox": (x, y, w, h)}, ...]
        """
        # Convertir a escala de grises
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        
        # Aplicar threshold para obtener bordes
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        
        # Encontrar contornos
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        candidates = []
        min_area = 50  # Área mínima para considerar un cuadrado
        max_area = 2000  # Área máxima (para excluir objetos muy grandes)
        
        for contour in contours:
            # Aproximar contorno a polígono
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # Si tiene 4 vértices, es probablemente un cuadrado/rectángulo
            if len(approx) == 4:
                area = cv2.contourArea(contour)
                if min_area < area < max_area:
                    # Calcular bounding box
                    x, y, w, h = cv2.boundingRect(contour)
                    
                    # Calcular centro
                    center_x = x + w // 2
                    center_y = y + h // 2
                    
                    # Filtrar si está muy cerca de los bordes
                    margin = 5
                    if (center_x < margin or center_x > self.image_width - margin or
                        center_y < margin or center_y > self.image_height - margin):
                        continue
                    
                    # Calcular relación de aspecto (cuadrados deberían tener ~1.0)
                    aspect_ratio = float(w) / h if h > 0 else 0
                    
                    # Filtrar si es muy rectangular (no cuadrado)
                    if aspect_ratio < 0.5 or aspect_ratio > 2.0:
                        continue
                    
                    candidates.append({
                        "x": center_x,
                        "y": center_y,
                        "width": w,
                        "height": h,
                        "bbox": (x, y, w, h),
                        "area": area,
                        "aspect_ratio": aspect_ratio,
                        "type": "square"
                    })
        
        return candidates
    
    def extract_candidate_region(self, image: np.ndarray, candidate: Dict) -> np.ndarray:
        """
        Extrae la región de un candidato de la imagen.
        
        Args:
            image: Imagen completa
            candidate: Diccionario con información del candidato
        
        Returns:
            Región recortada de la imagen
        """
        bbox = candidate.get("bbox")
        if bbox:
            x, y, w, h = bbox
            # Añadir padding
            padding = 5
            x = max(0, x - padding)
            y = max(0, y - padding)
            w = min(self.image_width - x, w + 2 * padding)
            h = min(self.image_height - y, h + 2 * padding)
            return image[y:y+h, x:x+w]
        return image
    
    def get_candidate_color_hint(self, image: np.ndarray, candidate: Dict) -> str:
        """
        Obtiene una pista del color del candidato usando análisis de color.
        
        Args:
            image: Imagen completa
            candidate: Diccionario con información del candidato
        
        Returns:
            Color dominante aproximado ("red", "orange", "green", etc.)
        """
        # Extraer región del candidato
        region = self.extract_candidate_region(image, candidate)
        if region.size == 0:
            return "unknown"
        
        # Convertir a HSV
        hsv = cv2.cvtColor(region, cv2.COLOR_RGB2HSV)
        
        # Calcular histograma de H (matiz)
        hist = cv2.calcHist([hsv], [0], None, [180], [0, 180])
        
        # Encontrar el matiz dominante
        dominant_h = np.argmax(hist)
        
        # Mapear a color aproximado
        if 0 <= dominant_h < 10 or 170 <= dominant_h < 180:
            return "red"
        elif 10 <= dominant_h < 25:
            return "orange"
        elif 40 <= dominant_h < 80:
            return "green"
        elif 100 <= dominant_h < 130:
            return "blue"
        elif 150 <= dominant_h < 170:
            return "magenta"
        elif 20 <= dominant_h < 30:
            return "yellow"
        else:
            return "unknown"
    
    def detect_victims_candidates(self, image: np.ndarray) -> List[Dict]:
        """
        Detecta candidatos a víctimas (círculos de colores).
        
        Args:
            image: Imagen RGB
        
        Returns:
            Lista de candidatos con información adicional
        """
        circles = self.detect_circles(image)
        
        # Agregar información de color a cada candidato
        for circle in circles:
            color_hint = self.get_candidate_color_hint(image, circle)
            circle["color_hint"] = color_hint
        
        return circles
    
    def detect_obstacles_candidates(self, image: np.ndarray) -> List[Dict]:
        """
        Detecta candidatos a obstáculos (cuadrados de colores).
        
        Args:
            image: Imagen RGB
        
        Returns:
            Lista de candidatos con información adicional
        """
        squares = self.detect_squares(image)
        
        # Filtrar el rover (cuadrado azul) - debería estar en una posición conocida
        # Por ahora, filtrar por color azul
        filtered_squares = []
        for square in squares:
            color_hint = self.get_candidate_color_hint(image, square)
            # Excluir azul (rover)
            if color_hint != "blue":
                square["color_hint"] = color_hint
                filtered_squares.append(square)
        
        return filtered_squares
    
    def create_candidates_summary(self, candidates: List[Dict], candidate_type: str) -> str:
        """
        Crea un resumen de candidatos para enviar a GPT.
        
        Args:
            candidates: Lista de candidatos detectados
            candidate_type: "victims" o "obstacles"
        
        Returns:
            Texto resumen formateado
        """
        if not candidates:
            return f"No se detectaron candidatos a {candidate_type}."
        
        summary = f"Se detectaron {len(candidates)} candidatos a {candidate_type}:\n\n"
        
        for i, candidate in enumerate(candidates, 1):
            summary += f"Candidato {i}:\n"
            summary += f"  - Posición (píxeles): ({candidate['x']}, {candidate['y']})\n"
            summary += f"  - Tipo: {candidate['type']}\n"
            
            if candidate_type == "victims":
                summary += f"  - Radio: {candidate.get('radius', 'N/A')} píxeles\n"
            else:
                summary += f"  - Tamaño: {candidate.get('width', 'N/A')}x{candidate.get('height', 'N/A')} píxeles\n"
            
            summary += f"  - Color dominante aproximado: {candidate.get('color_hint', 'unknown')}\n"
            summary += f"  - Bounding box: {candidate.get('bbox', 'N/A')}\n\n"
        
        return summary

