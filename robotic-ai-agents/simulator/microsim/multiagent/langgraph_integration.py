"""
Integración de LangGraph con ROS 2 para análisis de imágenes y toma de decisiones.
Este módulo adapta las funciones del UAV con imágenes de ROS 2.
"""

import os
import cv2
import base64
import json
import time
import re
from typing import Dict, List, Optional, Tuple, Any
import numpy as np
from cv_bridge import CvBridge

# Detección geométrica deshabilitada - usar solo GPT
GEOMETRIC_DETECTION_AVAILABLE = False

# Importar desde MultiAgent si está disponible
try:
    import sys
    multiagent_path = os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "MultiAgent")
    if os.path.exists(multiagent_path):
        sys.path.insert(0, multiagent_path)
    from langchain_openai import ChatOpenAI
    from dotenv import load_dotenv
    
    # Cargar .env desde MultiAgent
    multiagent_env_path = os.path.join(multiagent_path, ".env")
    load_dotenv(multiagent_env_path)
    
    llm = ChatOpenAI(model="gpt-5-mini", api_key=os.getenv("OPENAI_API_KEY"))
    llm5 = ChatOpenAI(model="gpt-5", api_key=os.getenv("OPENAI_API_KEY"))
except Exception as e:
    print(f"⚠️ Advertencia: No se pudo cargar LangChain. Algunas funciones no estarán disponibles: {e}")
    llm = None


class UAVState(dict):
    """
    Estado del agente UAV adaptado para ROS 2.
    Similar al State original pero adaptado para trabajar en tiempo real.
    """
    pass


def preprocess_image_for_detection(frame: np.ndarray) -> np.ndarray:
    """
    Preprocesa una imagen para mejorar la detección:
    - Mejora de contraste (CLAHE)
    - Reducción de ruido (filtro bilateral)
    - Normalización de brillo
    
    Args:
        frame: Frame de OpenCV (numpy array)
        
    Returns:
        Frame preprocesado
    """
    try:
        # Convertir a RGB si está en BGR
        if len(frame.shape) == 3 and frame.shape[2] == 3:
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        else:
            frame_rgb = frame.copy()
        
        # Aplicar CLAHE (Contrast Limited Adaptive Histogram Equalization) para mejorar contraste
        if len(frame_rgb.shape) == 3:
            lab = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2LAB)
            l, a, b = cv2.split(lab)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            l = clahe.apply(l)
            frame_rgb = cv2.merge([l, a, b])
            frame_rgb = cv2.cvtColor(frame_rgb, cv2.COLOR_LAB2RGB)
        
        # Reducción de ruido con filtro bilateral (preserva bordes)
        frame_rgb = cv2.bilateralFilter(frame_rgb, 5, 50, 50)
        
        return frame_rgb
    except Exception as e:
        print(f"⚠️ Error en preprocesamiento de imagen, usando original: {e}")
        return frame.copy() if len(frame.shape) == 3 else frame


def encode_frame_to_base64(frame: np.ndarray, frame_number: int = 0, 
                           use_preprocessing: bool = True, 
                           use_png: bool = False,
                           save_to_file: bool = True) -> str:
    """
    Codifica un frame de OpenCV a base64 para enviarlo a GPT.
    Incluye preprocesamiento opcional para mejorar la detección.
    
    Args:
        frame: Frame de OpenCV (numpy array)
        frame_number: Número de frame para logging
        use_preprocessing: Si True, aplica preprocesamiento de imagen
        use_png: Si True, usa PNG (sin pérdida), si False usa JPEG (más rápido)
        save_to_file: Si True, guarda el base64 en MultiAgent/frame_base64_1.txt
        
    Returns:
        String base64 codificado
    """
    try:
        # Preprocesar imagen si está habilitado
        if use_preprocessing:
            frame_processed = preprocess_image_for_detection(frame)
        else:
            if len(frame.shape) == 3 and frame.shape[2] == 3:
                frame_processed = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            else:
                frame_processed = frame
        
        # Codificar a PNG (sin pérdida) o JPEG (más rápido)
        if use_png:
            _, buffer = cv2.imencode('.png', frame_processed)
        else:
            # JPEG con mayor calidad para mejor detección
            _, buffer = cv2.imencode('.jpg', frame_processed, [cv2.IMWRITE_JPEG_QUALITY, 95])
        
        frame_base64 = base64.b64encode(buffer).decode('utf-8')
        
        # Guardar en archivo si está habilitado (imagen exacta que se envía a GPT)
        if save_to_file:
            try:
                # Buscar directorio MultiAgent
                current_dir = os.path.dirname(os.path.abspath(__file__))
                multiagent_path = os.path.join(current_dir, "..", "..", "..", "..", "MultiAgent")
                
                if os.path.exists(multiagent_path):
                    base64_filename = os.path.join(multiagent_path, "frame_base64_1.txt")
                    with open(base64_filename, "w", encoding="utf-8") as f:
                        f.write(frame_base64)
                    print(f"💾 Base64 guardado en: {base64_filename}")
                else:
                    # Intentar guardar en el directorio actual si no se encuentra MultiAgent
                    base64_filename = os.path.join(current_dir, "frame_base64_1.txt")
                    with open(base64_filename, "w", encoding="utf-8") as f:
                        f.write(frame_base64)
                    print(f"💾 Base64 guardado en: {base64_filename}")
            except Exception as e:
                print(f"⚠️ Error guardando base64 en archivo: {e}")
        
        return frame_base64
    except Exception as e:
        print(f"❌ Error codificando frame {frame_number}: {e}")
        return ""


def validate_coordinates(coords: Dict, width: int, height: int, 
                        min_distance: float = 5.0) -> bool:
    """
    Valida que las coordenadas estén dentro del rango válido de la imagen.
    
    Args:
        coords: Diccionario con 'x' e 'y'
        width: Ancho de la imagen
        height: Alto de la imagen
        min_distance: Distancia mínima desde los bordes (píxeles)
        
    Returns:
        True si las coordenadas son válidas
    """
    x = coords.get('x', -1)
    y = coords.get('y', -1)
    
    return (min_distance <= x <= width - min_distance and 
            min_distance <= y <= height - min_distance)


def filter_duplicate_detections(detections: List[Dict], 
                                 min_distance_pixels: float = 10.0) -> List[Dict]:
    """
    Filtra detecciones duplicadas que están muy cerca entre sí.
    Mantiene la detección con mayor confianza o la primera encontrada.
    
    Args:
        detections: Lista de detecciones con coordenadas 'x' e 'y'
        min_distance_pixels: Distancia mínima en píxeles para considerar duplicados
        
    Returns:
        Lista filtrada de detecciones
    """
    if not detections:
        return []
    
    filtered = []
    for detection in detections:
        coords = detection.get('coordenadas', {})
        x = coords.get('x', 0)
        y = coords.get('y', 0)
        
        is_duplicate = False
        for existing in filtered:
            existing_coords = existing.get('coordenadas', {})
            ex = existing_coords.get('x', 0)
            ey = existing_coords.get('y', 0)
            
            distance = np.sqrt((x - ex)**2 + (y - ey)**2)
            if distance < min_distance_pixels:
                is_duplicate = True
                break
        
        if not is_duplicate:
            filtered.append(detection)
    
    return filtered


def extract_json_from_response(response_text: str) -> Optional[Dict]:
    """
    Extrae JSON de una respuesta de GPT de forma más robusta.
    Intenta múltiples métodos para encontrar el JSON válido.
    
    Args:
        response_text: Texto de respuesta de GPT
        
    Returns:
        Diccionario con el JSON parseado, o None si falla
    """
    if not response_text:
        return None
    
    # Método 1: Buscar entre llaves simples
    json_start = response_text.find('{')
    json_end = response_text.rfind('}') + 1
    
    if json_start != -1 and json_end > json_start:
        try:
            json_str = response_text[json_start:json_end]
            result = json.loads(json_str)
            return result
        except json.JSONDecodeError:
            pass
    
    # Método 2: Buscar bloques de código JSON
    json_pattern = r'```(?:json)?\s*(\{.*?\})\s*```'
    matches = re.findall(json_pattern, response_text, re.DOTALL)
    if matches:
        try:
            result = json.loads(matches[0])
            return result
        except json.JSONDecodeError:
            pass
    
    # Método 3: Buscar cualquier estructura JSON válida
    json_pattern = r'\{[^{}]*(?:\{[^{}]*\}[^{}]*)*\}'
    matches = re.findall(json_pattern, response_text, re.DOTALL)
    for match in matches:
        try:
            result = json.loads(match)
            if isinstance(result, dict):
                return result
        except json.JSONDecodeError:
            continue
    
    return None


def classify_victims_with_gpt(frame_base64: str, candidates: List[Dict], 
                               width: int, height: int) -> Dict:
    """
    Clasifica candidatos a víctimas usando GPT para validar y clasificar color/estado.
    
    Args:
        frame_base64: Frame codificado en base64
        candidates: Lista de candidatos detectados geométricamente
        width: Ancho de la imagen
        height: Alto de la imagen
    
    Returns:
        Diccionario con víctimas clasificadas
    """
    if llm is None:
        return {"victimas_identificadas": [], "total_victimas": 0}
    
    # Crear resumen de candidatos
    candidates_text = ""
    for i, candidate in enumerate(candidates, 1):
        candidates_text += f"Candidato {i}: Posición ({candidate['x']}, {candidate['y']}), "
        candidates_text += f"Radio {candidate.get('radius', 'N/A')}px, "
        candidates_text += f"Color sugerido: {candidate.get('color_hint', 'unknown')}\n"
    
    prompt = f"""
    Eres un sistema de clasificación de víctimas para rescate en desastres.
    
    Se han detectado geométricamente {len(candidates)} candidatos a víctimas (círculos de colores).
    Tu tarea es VALIDAR cada candidato y CLASIFICAR su color/estado.
    
    INFORMACIÓN DE LA IMAGEN:
    - Resolución: {width} x {height} píxeles
    
    CANDIDATOS DETECTADOS:
    {candidates_text}
    
    INSTRUCCIONES:
    1. Para CADA candidato, verifica si es realmente una víctima (círculo de color)
    2. Clasifica el color exacto: ROJO, NARANJA, o VERDE OSCURO
    3. Determina el estado según el color:
       - ROJO = crítico (prioridad alta)
       - NARANJA = herido (prioridad media)
       - VERDE OSCURO = seguro (prioridad baja)
    4. Usa las coordenadas EXACTAS proporcionadas (centro del círculo detectado)
    5. Si un candidato NO es una víctima (es un obstáculo u otro objeto), EXCLUYELO
    
    Responde en formato JSON:
    {{
        "victimas_identificadas": [
            {{
                "id": 1,
                "coordenadas": {{"x": 150, "y": 200}},
                "estado": "crítico/herido/seguro",
                "color": "rojo/naranja/verde oscuro",
                "prioridad": "alta/media/baja",
                "validado": true
            }}
        ],
        "total_victimas": número_total,
        "candidatos_rechazados": número_rechazados
    }}
    """
    
    try:
        message = {
            "role": "user",
            "content": [
                {"type": "text", "text": prompt},
                {
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:image/png;base64,{frame_base64}",
                        "detail": "high"
                    }
                }
            ]
        }
        
        response = llm.invoke([message])
        analysis_result = response.content
        
        # Extraer JSON
        json_start = analysis_result.find('{')
        json_end = analysis_result.rfind('}') + 1
        
        if json_start != -1 and json_end != -1:
            json_str = analysis_result[json_start:json_end]
            result = json.loads(json_str)
            return result
        else:
            print("⚠️ No se encontró JSON válido en la respuesta de clasificación")
            return {"victimas_identificadas": [], "total_victimas": 0}
            
    except Exception as e:
        print(f"❌ Error en clasificación de víctimas: {e}")
        return {"victimas_identificadas": [], "total_victimas": 0}


def identify_victims_from_image(frame: np.ndarray, frame_base64: str = None, 
                                  current_position: Optional[tuple] = None,
                                  use_preprocessing: bool = True) -> Dict:
    """
    Identifica víctimas en un frame usando GPT-4o Vision.
    
    Args:
        frame: Frame de OpenCV
        frame_base64: Frame codificado en base64
        current_position: Posición actual del UAV (x, y, z) en metros
        
    Returns:
        Diccionario con víctimas identificadas
    """
    if llm is None:
        return {"victimas_identificadas": [], "total_victimas": 0}
    
    height, width = frame.shape[:2]
    
    # Recodificar frame con preprocesamiento para mejor detección
    # Guardar solo la primera vez (para víctimas) para evitar duplicados
    if use_preprocessing or frame_base64 is None:
        frame_base64 = encode_frame_to_base64(frame, use_preprocessing=True, use_png=False, save_to_file=True)
    
    prompt = f"""
    Eres un sistema de análisis de video aéreo para rescate en desastres. 
    
    ⚠️ CRÍTICO: PRECISIÓN ES FUNDAMENTAL
    Tómate TODO el tiempo necesario para analizar la imagen cuidadosamente. 
    La precisión en las coordenadas es MÁS IMPORTANTE que la velocidad del análisis.
    
    INFORMACIÓN DE LA IMAGEN:
    - Resolución: {width} x {height} píxeles
    
    ANALIZA ESTE FRAME ESPECÍFICAMENTE PARA IDENTIFICAR VÍCTIMAS:
    
    - Las víctimas aparecen como CÍRCULOS de colores:
      * ROJO (#FF0000, RGB: 255, 0, 0): Víctima crítica (necesita rescate inmediato)
      * NARANJA (#FF6700, RGB: 255, 103, 0): Víctima herida (necesita atención médica)
      * VERDE OSCURO (#006400, RGB: 0, 100, 0): Víctima segura (estable)
    
    - Los obstáculos son figuras cuadradas de colores:
      * MARRÓN (#8B4513, RGB: 139, 69, 19) = edificio
      * MAGENTA (#FF00FF, RGB: 255, 0, 255) = escombro
      * AMARILLO (#C8AA3C, RGB: 200, 170, 60) = árbol
      * GRIS OSCURO (#404040, RGB: 64, 64, 64) = vehículo
      - NO son víctimas
    - El fondo es marrón claro con cielo azul
    
    INSTRUCCIONES DETALLADAS:
    1. EXAMINA la imagen con MUCHO cuidado. Identifica CADA círculo de color.
       - NO confundas los obstáculos cuadrados con víctimas
       - NO omitas ninguna víctima, incluso si está parcialmente visible
    2. Para CADA víctima identificada:
       a. Localiza el CENTRO EXACTO del círculo con máxima precisión
       b. NO uses el borde del círculo, sino el PUNTO CENTRAL exacto
       c. Examina cuidadosamente la imagen para determinar las coordenadas pixel (x, y) del centro
       d. Usa el sistema de coordenadas donde (0,0) es la esquina superior izquierda
    3. PRECISIÓN EN COORDENADAS:
       - Las coordenadas que proporciones se convertirán al sistema del mundo
       - Cada píxel de error puede resultar en metros de error en el mundo real
       - El rover depende de estas coordenadas para llegar a las víctimas
       - Tómate el tiempo necesario para medir las coordenadas con precisión máxima
    4. Determina el estado de cada víctima según su color EXACTO:
      - ROJO (#FF0000, RGB: 255, 0, 0) = crítico
      - NARANJA (#FF6700, RGB: 255, 103, 0) = herido
      - VERDE OSCURO (#006400, RGB: 0, 100, 0) = seguro
    
    Responde en formato JSON:
    {{
        "victimas_identificadas": [
            {{
                "id": 1,
                "coordenadas": {{"x": 150, "y": 200}},
                "estado": "crítico/herido/atrapado/seguro",
                "color": "rojo/naranja/morado/verde",
                "prioridad": "alta/media/baja"
            }}
        ],
        "total_victimas": número_total,
        "resolucion_imagen": {{
            "width": {width},
            "height": {height}
        }}
    }}
    """
    
    try:
        # Enviar imagen y prompt a GPT-4o Vision
        message = {
            "role": "user",
            "content": [
                {"type": "text", "text": prompt},
                {
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:image/png;base64,{frame_base64}",
                        "detail": "high"
                    }
                }
            ]
        }
        
        response = llm.invoke([message])
        analysis_result = response.content
        
        # Extraer JSON de la respuesta (método mejorado)
        result = extract_json_from_response(analysis_result)
        
        if result is None:
            print("⚠️ No se encontró JSON válido en la respuesta de víctimas")
            return {"victimas_identificadas": [], "total_victimas": 0}
        
        # Validar y filtrar coordenadas
        victimas_validas = []
        for victima in result.get("victimas_identificadas", []):
            coords = victima.get("coordenadas", {})
            if validate_coordinates(coords, width, height):
                victimas_validas.append(victima)
            else:
                print(f"⚠️ Coordenadas inválidas descartadas: {coords}")
        
        # Filtrar duplicados
        victimas_validas = filter_duplicate_detections(victimas_validas)
        
        result["victimas_identificadas"] = victimas_validas
        result["total_victimas"] = len(victimas_validas)
        
        return result
            
    except Exception as e:
        print(f"❌ Error en identificación de víctimas: {e}")
        import traceback
        traceback.print_exc()
        return {"victimas_identificadas": [], "total_victimas": 0}


def classify_obstacles_with_gpt(frame_base64: str, candidates: List[Dict],
                                 width: int, height: int) -> Dict:
    """
    Clasifica candidatos a obstáculos usando GPT para validar y clasificar tipo/color.
    
    Args:
        frame_base64: Frame codificado en base64
        candidates: Lista de candidatos detectados geométricamente
        width: Ancho de la imagen
        height: Alto de la imagen
    
    Returns:
        Diccionario con obstáculos clasificados
    """
    if llm is None:
        return {"obstaculos_identificados": [], "total_obstaculos": 0}
    
    # Crear resumen de candidatos
    candidates_text = ""
    for i, candidate in enumerate(candidates, 1):
        candidates_text += f"Candidato {i}: Posición ({candidate['x']}, {candidate['y']}), "
        candidates_text += f"Tamaño {candidate.get('width', 'N/A')}x{candidate.get('height', 'N/A')}px, "
        candidates_text += f"Color sugerido: {candidate.get('color_hint', 'unknown')}\n"
    
    prompt = f"""
    Eres un sistema de clasificación de obstáculos para rescate en desastres.
    
    Se han detectado geométricamente {len(candidates)} candidatos a obstáculos (cuadrados de colores).
    Tu tarea es VALIDAR cada candidato y CLASIFICAR su tipo según el color.
    
    INFORMACIÓN DE LA IMAGEN:
    - Resolución: {width} x {height} píxeles
    
    CANDIDATOS DETECTADOS:
    {candidates_text}
    
    IMPORTANTE: El ROVER es un cuadrado AZUL con borde negro - NO debe ser identificado como obstáculo.
    
    INSTRUCCIONES:
    1. Para CADA candidato, verifica si es realmente un obstáculo (cuadrado de color)
    2. EXCLUYE el rover (cuadrado azul con borde negro) si está presente
    3. Clasifica el tipo según el color:
       - MARRÓN = edificio/estructura
       - MAGENTA/ROSA = escombro
       - AMARILLO = árbol
       - GRIS = vehículo
    4. Usa las coordenadas EXACTAS proporcionadas (centro del cuadrado detectado)
    5. Si un candidato NO es un obstáculo válido, EXCLUYELO
    
    Responde en formato JSON:
    {{
        "obstaculos_identificados": [
            {{
                "id": 1,
                "coordenadas": {{"x": 150, "y": 200}},
                "tipo": "edificio/escombro/árbol/vehículo",
                "color": "marrón/magenta/azul/gris",
                "forma": "cuadrado",
                "validado": true
            }}
        ],
        "total_obstaculos": número_total,
        "candidatos_rechazados": número_rechazados
    }}
    """
    
    try:
        message = {
            "role": "user",
            "content": [
                {"type": "text", "text": prompt},
                {
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:image/png;base64,{frame_base64}",
                        "detail": "high"
                    }
                }
            ]
        }
        
        response = llm.invoke([message])
        analysis_result = response.content
        
        # Extraer JSON
        json_start = analysis_result.find('{')
        json_end = analysis_result.rfind('}') + 1
        
        if json_start != -1 and json_end != -1:
            json_str = analysis_result[json_start:json_end]
            result = json.loads(json_str)
            return result
        else:
            print("⚠️ No se encontró JSON válido en la respuesta de clasificación de obstáculos")
            return {"obstaculos_identificados": [], "total_obstaculos": 0}
            
    except Exception as e:
        print(f"❌ Error en clasificación de obstáculos: {e}")
        return {"obstaculos_identificados": [], "total_obstaculos": 0}


def identify_obstacles_from_image(frame: np.ndarray, frame_base64: str = None,
                                   use_preprocessing: bool = True) -> Dict:
    """
    Identifica obstáculos en un frame usando GPT-4o Vision.
    
    Args:
        frame: Frame de OpenCV
        frame_base64: Frame codificado en base64
    
    Returns:
        Diccionario con obstáculos identificados
    """
    if llm is None:
        return {"obstaculos_identificados": [], "total_obstaculos": 0}
    
    height, width = frame.shape[:2]
    
    # Recodificar frame con preprocesamiento para mejor detección
    # No guardar aquí (ya se guardó en identify_victims_from_image)
    if use_preprocessing or frame_base64 is None:
        frame_base64 = encode_frame_to_base64(frame, use_preprocessing=True, use_png=False, save_to_file=False)
    
    prompt = f"""
    Eres un sistema de análisis de video aéreo para rescate en desastres.
    
    ⚠️ CRÍTICO: PRECISIÓN ES FUNDAMENTAL
    Tómate TODO el tiempo necesario para analizar la imagen cuidadosamente. 
    La precisión en las coordenadas es MÁS IMPORTANTE que la velocidad del análisis.
    
    INFORMACIÓN DE LA IMAGEN:
    - Resolución: {width} x {height} píxeles
    
    ANALIZA ESTE FRAME ESPECÍFICAMENTE PARA IDENTIFICAR OBSTÁCULOS:
    
    - Los obstáculos son figuras CUADRADAS de color sólido:
      * MARRÓN (#8B4513, RGB: 139, 69, 19): Edificios/estructuras
      * MAGENTA (#FF00FF, RGB: 255, 0, 255): Escombros
      * AMARILLO (#C8AA3C, RGB: 200, 170, 60): Árboles
      * GRIS OSCURO (#404040, RGB: 64, 64, 64): Vehículos
    
    - IMPORTANTE: El ROVER es un CUADRADO AZUL (#0000FF, RGB: 0, 0, 255) CON BORDE NEGRO - NO es un obstáculo
    - Las víctimas son CÍRCULOS de colores - NO son obstáculos
    - El fondo es marrón claro con cielo azul
    
    INSTRUCCIONES DETALLADAS:
    1. EXAMINA la imagen con MUCHO cuidado. Identifica CADA figura cuadrada que sea un obstáculo.
       - NO confundas las víctimas (círculos) con obstáculos (cuadrados)
       - NO incluyas el rover (cuadrado azul con borde negro) como obstáculo
       - NO omitas ningún obstáculo, incluso si está parcialmente visible
    2. EXCLUYE explícitamente:
       - El ROVER (cuadrado azul con borde negro) - NO es un obstáculo
       - Las VÍCTIMAS (círculos de colores) - NO son obstáculos
    3. Para CADA obstáculo identificado:
       a. Localiza el CENTRO EXACTO del cuadrado con máxima precisión
       b. NO uses una esquina del cuadrado, sino el PUNTO CENTRAL exacto del cuadrado
       c. Examina cuidadosamente la imagen para determinar las coordenadas pixel (x, y) del centro
       d. Usa el sistema de coordenadas donde (0,0) es la esquina superior izquierda
    4. PRECISIÓN EN COORDENADAS:
       - Las coordenadas que proporciones se convertirán al sistema del mundo
       - Cada píxel de error puede resultar en metros de error en el mundo real
       - El rover depende de estas coordenadas para EVITAR los obstáculos correctamente
       - Tómate el tiempo necesario para medir las coordenadas con precisión máxima
    5. Determina el tipo de obstáculo según su color EXACTO:
      - MARRÓN (#8B4513, RGB: 139, 69, 19) = edificio
      - MAGENTA (#FF00FF, RGB: 255, 0, 255) = escombro
      - AMARILLO (#C8AA3C, RGB: 200, 170, 60) = árbol
      - GRIS OSCURO (#404040, RGB: 64, 64, 64) = vehículo
    
    Responde en formato JSON:
    {{
        "obstaculos_identificados": [
            {{
                "id": 1,
                "coordenadas": {{"x": 150, "y": 200}},
                "tipo": "edificio/escombro/árbol/vehículo",
                "forma": "rectángulo/triángulo/hexágono",
                "color": "marrón/magenta/azul/amarillo",
                "tamaño": "pequeño/mediano/grande"
            }}
        ],
        "total_obstaculos": número_total,
        "resolucion_imagen": {{
            "width": {width},
            "height": {height}
        }}
    }}
    """
    
    try:
        # CORRECCIÓN: Enviar imagen junto con el prompt (igual que en identify_victims_from_image)
        message = {
            "role": "user",
            "content": [
                {"type": "text", "text": prompt},
                {
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:image/png;base64,{frame_base64}",
                        "detail": "high"
                    }
                }
            ]
        }
        
        response = llm.invoke([message])
        analysis_result = response.content
        
        # Extraer JSON de la respuesta (método mejorado)
        result = extract_json_from_response(analysis_result)
        
        if result is None:
            print("⚠️ No se encontró JSON válido en la respuesta de obstáculos")
            return {"obstaculos_identificados": [], "total_obstaculos": 0}
        
        # Validar y filtrar coordenadas
        obstaculos_validos = []
        for obstaculo in result.get("obstaculos_identificados", []):
            coords = obstaculo.get("coordenadas", {})
            if validate_coordinates(coords, width, height):
                obstaculos_validos.append(obstaculo)
            else:
                print(f"⚠️ Coordenadas inválidas descartadas: {coords}")
        
        # Filtrar duplicados
        obstaculos_validos = filter_duplicate_detections(obstaculos_validos)
        
        result["obstaculos_identificados"] = obstaculos_validos
        result["total_obstaculos"] = len(obstaculos_validos)
        
        return result
            
    except Exception as e:
        print(f"❌ Error en identificación de obstáculos: {e}")
        import traceback
        traceback.print_exc()
        return {"obstaculos_identificados": [], "total_obstaculos": 0}


def plan_route(victims: List[Dict], obstacles: List[Dict], 
                start_position: tuple = (0.0, 0.0, 15.0)) -> Dict:
    """
    Planifica una ruta óptima usando GPT.
    
    Args:
        victims: Lista de víctimas identificadas
        obstacles: Lista de obstáculos identificados
        start_position: Posición inicial del UAV (x, y, z)
        
    Returns:
        Diccionario con ruta optimizada
    """
    if llm is None:
        return {"ruta_optimizada": None}
    
    # Formatear datos para el prompt
    victims_data = []
    for i, victim in enumerate(victims, 1):
        coords = victim.get("coordenadas", {})
        x = coords.get("x", 0)
        y = coords.get("y", 0)
        estado = victim.get("estado", "desconocido")
        prioridad = victim.get("prioridad", "media")
        victims_data.append(f"Víctima {i}: Posición=({x}, {y}), Estado={estado}, Prioridad={prioridad}")
    
    obstacles_data = []
    for i, obstacle in enumerate(obstacles, 1):
        coords = obstacle.get("coordenadas", {})
        x = coords.get("x", 0)
        y = coords.get("y", 0)
        tipo = obstacle.get("tipo", "desconocido")
        obstacles_data.append(f"Obstáculo {i}: Posición=({x}, {y}), Tipo={tipo}")
    
    prompt = f"""
    Eres un sistema de planificación de rutas para rescate en desastres. 
    
    ⚠️ CRÍTICO: PRECISIÓN Y COMPLETITUD SON FUNDAMENTALES
    Tómate TODO el tiempo necesario para planificar la ruta cuidadosamente. 
    La calidad de la ruta es MÁS IMPORTANTE que la velocidad del análisis.
    
    OBJETIVO PRINCIPAL: Planificar UNA SOLA ruta óptima desde la posición inicial {start_position} que:
    - Pase por TODAS las víctimas (NO puedes omitir ninguna)
    - Evite TODOS los obstáculos (manteniendo distancia de seguridad)
    - Minimice la distancia total recorrida
    - Priorice las víctimas críticas primero
    
    VÍCTIMAS A VISITAR (TODAS - OBLIGATORIO VISITAR CADA UNA):
    {chr(10).join(victims_data) if victims_data else "  No hay víctimas identificadas"}
    
    OBSTÁCULOS A EVITAR (CRÍTICO - MANTENER DISTANCIA):
    {chr(10).join(obstacles_data) if obstacles_data else "  No hay obstáculos identificados"}
    
    ⚠️ REGLAS CRÍTICAS DE PLANIFICACIÓN:
    1. COMPLETITUD OBLIGATORIA:
       - La ruta DEBE pasar por TODAS las víctimas listadas arriba
       - NO puedes omitir ninguna víctima, sin importar su prioridad
       - Cada víctima DEBE estar incluida en la ruta como un punto de paso obligatorio
    
    2. EVITAR OBSTÁCULOS (CRÍTICO):
       - La ruta DEBE evitar TODOS los obstáculos listados
       - Mantén una distancia de seguridad de al menos 3-5 metros de cada obstáculo
       - Si un obstáculo bloquea el camino directo a una víctima, planifica un desvío alrededor del obstáculo
       - NO pases a través de obstáculos, SIEMPRE pasa alrededor
    
    3. ORDEN DE PRIORIDAD:
       - Prioriza visitar primero las víctimas con estado "crítico" o prioridad "alta"
       - Luego visita las víctimas con estado "herido" o prioridad "media"
       - Finalmente visita las víctimas con estado "seguro" o prioridad "baja"
    
    4. OPTIMIZACIÓN:
       - Minimiza la distancia total recorrida
       - Minimiza el número de cambios de dirección innecesarios
       - Pero SIEMPRE prioriza la seguridad (evitar obstáculos) sobre la distancia
    
    5. PUNTOS DE PASO:
       - Incluye puntos de paso intermedios si son necesarios para evitar obstáculos
       - Incluye el punto de inicio de la ruta
       - Incluye el punto final después de visitar todas las víctimas
    
    ESTRUCTURA DE RESPUESTA:
    - Proporciona una ruta secuencial con todos los puntos de paso
    - Cada punto debe tener coordenadas (x, y) en píxeles
    - Indica el tipo de cada punto: "inicio", "victima", "obstaculo_evitado", "punto_paso"
    - Si es una víctima, incluye el ID de la víctima
    
    Responde en formato JSON con la estructura de ruta optimizada completa.
    """
    
    try:
        message = {
            "role": "user",
            "content": prompt
        }
        
        response = llm.invoke([message])
        route_analysis = response.content
        
        # Extraer JSON
        json_start = route_analysis.find('{')
        json_end = route_analysis.rfind('}') + 1
        
        if json_start != -1 and json_end != -1:
            json_str = route_analysis[json_start:json_end]
            route_data = json.loads(json_str)
            return route_data
        else:
            print("⚠️ No se encontró JSON válido en la respuesta de planificación")
            return {"ruta_optimizada": None}
            
    except Exception as e:
        print(f"❌ Error en planificación de ruta: {e}")
        return {"ruta_optimizada": None}


def convert_image_to_cv2(image_msg) -> Optional[np.ndarray]:
    """
    Convierte un mensaje Image de ROS 2 a un array de OpenCV.
    
    Args:
        image_msg: sensor_msgs.msg.Image
        
    Returns:
        Frame de OpenCV o None si hay error
    """
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding='rgb8')
        return cv_image
    except Exception as e:
        print(f"❌ Error convirtiendo imagen ROS 2 a OpenCV: {e}")
        return None

