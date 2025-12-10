import os
import cv2
import base64
import json
import time
import math
import sys
import numpy as np
from dotenv import load_dotenv

# Cargar .env desde el directorio actual (MultiAgent)
env_path = os.path.join(os.path.dirname(__file__), "..", ".env")
load_dotenv(env_path)

from langgraph.graph import StateGraph, END
from langchain_openai import ChatOpenAI

# Configurar LLM
llm = ChatOpenAI(model="gpt-5", api_key=os.getenv("OPENAI_API_KEY"))

# Detección geométrica deshabilitada - usar solo GPT
GEOMETRIC_DETECTION_AVAILABLE = False

class State(dict):
    video_path: str
    current_frame: int
    total_frames: int
    frame_data: object  # Frame de OpenCV
    frame_base64: str   # Frame codificado en base64
    analysis_result: str
    video_cap: object  # Para mantener la referencia del video
    max_frames_to_process: int  # Límite de frames a procesar
    victims_found: list  # Lista de víctimas encontradas
    obstacles_found: list  # Lista de obstáculos encontrados
    routes_planned: list  # Lista de rutas planificadas
    mission_brief: str  # Mensaje de misión para el agente terrestre
    communication_log: list  # Log de comunicaciones
    # Campos del controlador de movimiento
    uav_position: tuple  # Posición actual del UAV (x, y)
    patrol_route: list  # Ruta de patrullaje circular
    patrol_index: int  # Índice actual en la ruta de patrullaje
    patrol_complete: bool  # Indica si completó el patrullaje
    movement_speed: float  # Velocidad de movimiento (píxeles por comando)

def inicializadorPatrullaje(state: State) -> State:
    """Inicializa la ruta de patrullaje circular del UAV"""
    print("🛸 UAV: Inicializando sistema de patrullaje...")
    
    # Obtener dimensiones del área (del primer frame o usar valores por defecto)
    cap = state["video_cap"]
    ret, frame = cap.read()
    if ret:
        height, width = frame.shape[:2]
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Volver al inicio
    else:
        width, height = 640, 480  # Valores por defecto
    
    # Generar ruta circular que cubre toda la zona
    patrol_route = _generate_circular_patrol_route(width, height, start_pos=(10, 10))
    
    state["uav_position"] = (10, 10)
    state["patrol_route"] = patrol_route
    state["patrol_index"] = 0
    state["patrol_complete"] = False
    state["movement_speed"] = 3.0  # píxeles por comando
    
    print(f"🗺️ UAV: Ruta de patrullaje generada")
    print(f"   └─ Área de cobertura: {width}x{height} píxeles")
    print(f"   └─ Puntos de patrullaje: {len(patrol_route)}")
    print(f"   └─ Posición inicial: (10, 10)")
    print(f"   └─ Velocidad: {state['movement_speed']} px/cmd")
    
    return state

def _generate_circular_patrol_route(width: int, height: int, start_pos: tuple) -> list:
    """Genera una ruta circular que cubre el perímetro de la zona"""
    route = []
    
    # Definir puntos del perímetro en sentido horario
    margin = 20  # Margen desde los bordes
    
    # Esquina superior izquierda (inicio)
    route.append((margin, margin))
    
    # Lado superior (izquierda a derecha)
    for x in range(margin, width - margin, 50):
        route.append((x, margin))
    route.append((width - margin, margin))  # Esquina superior derecha
    
    # Lado derecho (arriba a abajo)
    for y in range(margin, height - margin, 50):
        route.append((width - margin, y))
    route.append((width - margin, height - margin))  # Esquina inferior derecha
    
    # Lado inferior (derecha a izquierda)
    for x in range(width - margin, margin, -50):
        route.append((x, height - margin))
    route.append((margin, height - margin))  # Esquina inferior izquierda
    
    # Lado izquierdo (abajo a arriba)
    for y in range(height - margin, margin, -50):
        route.append((margin, y))
    
    # Volver al punto de inicio
    route.append(start_pos)
    
    return route

def controladorPatrullaje(state: State) -> State:
    """Controlador de movimiento que ejecuta el patrullaje circular"""
    if state.get("patrol_complete", False):
        return state
    
    current_pos = state.get("uav_position", (10, 10))
    patrol_route = state.get("patrol_route", [])
    patrol_index = state.get("patrol_index", 0)
    movement_speed = state.get("movement_speed", 3.0)
    
    if patrol_index >= len(patrol_route):
        # Patrullaje completado
        state["patrol_complete"] = True
        print(f"✅ UAV: Patrullaje completado - Retornado a posición inicial")
        print(f"=" * 60)
        return state
    
    # Obtener siguiente punto objetivo
    target_point = patrol_route[patrol_index]
    current_x, current_y = current_pos
    target_x, target_y = target_point
    
    # Calcular distancia al objetivo
    dx = target_x - current_x
    dy = target_y - current_y
    distance = math.sqrt(dx**2 + dy**2)
    
    # Verificar si hemos llegado al punto
    if distance <= movement_speed:
        # Llegamos al punto
        state["uav_position"] = target_point
        state["patrol_index"] = patrol_index + 1
        print(f"📍 UAV: Punto {patrol_index + 1}/{len(patrol_route)} alcanzado - ({target_x}, {target_y})")
    else:
        # Calcular siguiente posición (movimiento en línea recta)
        direction_x = dx / distance
        direction_y = dy / distance
        
        new_x = current_x + direction_x * movement_speed
        new_y = current_y + direction_y * movement_speed
        
        state["uav_position"] = (int(new_x), int(new_y))
        
        # Mostrar comando de movimiento (menos frecuente para no saturar)
        if patrol_index % 2 == 0:  # Mostrar cada 2 puntos
            print(f"✈️ UAV: Patrullando hacia punto {patrol_index + 1}/{len(patrol_route)} - Posición: ({int(new_x)}, {int(new_y)})")
    
    return state

def procesadorVideo(state: State) -> State:
    """Obtiene el siguiente frame del video"""
    cap = state["video_cap"]
    current_frame = state["current_frame"]
    max_frames = state["max_frames_to_process"]
    
    # Leer el siguiente frame
    ret, frame = cap.read()
    
    if not ret or current_frame >= max_frames:
        print("🎬 Límite de frames alcanzado")
        state["current_frame"] = max_frames  # Forzar parada
        return state
    
    # Obtener directorio donde está UAV_agent.py (architecture/agents)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Guardar frame para visualización en architecture/agents
    frame_filename = os.path.join(script_dir, f"frame_analysis_{current_frame + 1}.png")
    cv2.imwrite(frame_filename, frame)
    
    # Codificar frame para GPT
    frame_base64 = encode_frame_to_base64(frame, current_frame + 1)
    
    # Guardar codificación base64 en archivo txt en architecture/agents
    base64_filename = os.path.join(script_dir, f"frame_base64_{current_frame + 1}.txt")
    try:
        with open(base64_filename, "w", encoding="utf-8") as f:
            f.write(frame_base64)
        print(f"💾 Base64 guardado en: {base64_filename}")
    except Exception as e:
        print(f"⚠️ Error guardando base64: {e}")
    
    state["frame_data"] = frame
    state["frame_base64"] = frame_base64
    state["current_frame"] = current_frame + 1
    
    return state

def identificacionVictimas(state: State) -> State:
    """Analiza el frame para identificar víctimas específicamente"""
    frame_base64 = state["frame_base64"]
    current_frame = state["current_frame"]
    frame_data = state["frame_data"]
    
    print(f"🔍 Analizando Frame {current_frame} para identificar víctimas...")
    
    # Obtener dimensiones de la imagen
    height, width = frame_data.shape[:2]
    
    # Prompt específico para identificación de víctimas con coordenadas
    prompt = f"""
    Eres un sistema de análisis de video aéreo para rescate en desastres. 
    
    INFORMACIÓN DE LA IMAGEN:
    - Resolución: {width} x {height} píxeles
    - Ancho: {width} píxeles
    - Alto: {height} píxeles
    
    SISTEMA DE COORDENADAS:
    - SISTEMA DE PÍXELES (imagen): 
      * (0, 0) = esquina superior izquierda de la imagen
      * ({width}, {height}) = esquina inferior derecha de la imagen
      * Centro de imagen: ({width//2}, {height//2}) píxeles
    
    - SISTEMA DE COORDENADAS DEL MUNDO:
      * El mundo tiene 100x100 metros
      * Rango del mundo: X desde -50 a +50 metros, Y desde -50 a +50 metros
      * Centro del mundo (0, 0) corresponde al centro de la imagen ({width//2}, {height//2}) píxeles
      * Posición del rover (punto de referencia): (40, 40) metros en coordenadas del mundo
      * Conversión aproximada: 1 metro del mundo ≈ {width/100:.1f} píxeles de la imagen
      * El píxel ({width//2}, {height//2}) corresponde a (0, 0) en coordenadas del mundo
      * Esquina superior izquierda ({0}, {0}) corresponde aproximadamente a (-50, -50) metros
      * Esquina inferior derecha ({width}, {height}) corresponde aproximadamente a (+50, +50) metros
    
    ANALIZA ESTE FRAME ESPECÍFICAMENTE PARA IDENTIFICAR VÍCTIMAS:
    
    - Las víctimas aparecen como CÍRCULOS de colores con bordes negros:
      * ROJO (#FF0000, RGB: 255, 0, 0): Víctima crítica (necesita rescate inmediato)
      * NARANJA (#FF6700, RGB: 255, 103, 0): Víctima herida (necesita atención médica)
      * VERDE OSCURO (#006400, RGB: 0, 100, 0): Víctima segura (estable)
    
    - Los obstáculos son figuras cuadradas más grandes de colores:
      * MARRÓN (#8B4513, RGB: 139, 69, 19) = edificio
      * MAGENTA (#FF00FF, RGB: 255, 0, 255) = escombro
      * AMARILLO (#C8AA3C, RGB: 200, 170, 60) = árbol
      * GRIS OSCURO (#404040, RGB: 64, 64, 64) = vehículo
      - NO son víctimas
    - El fondo es marrón claro con cielo azul
    
    ⚠️ CRÍTICO: PRECISIÓN ES FUNDAMENTAL
    Tómate TODO el tiempo necesario para analizar la imagen cuidadosamente. 
    La precisión en las coordenadas es MÁS IMPORTANTE que la velocidad del análisis.
    
    INSTRUCCIONES DETALLADAS:
    1. EXAMINA la imagen con MUCHO cuidado. Identifica CADA círculo de color rojo, naranja o verde oscuro.
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
      - ROJO (#FF0000, RGB: 255, 0, 0) = crítico (prioridad ALTA - necesita rescate inmediato)
      - NARANJA (#FF6700, RGB: 255, 103, 0) = herido (prioridad MEDIA - necesita atención médica)
      - VERDE OSCURO (#006400, RGB: 0, 100, 0) = seguro (prioridad BAJA - estable)
    5. Si un círculo está parcialmente fuera del frame:
       - Estima el centro visible con la mayor precisión posible
       - Indica en las coordenadas el punto central estimado del círculo completo
    
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
        "resolucion_imagen": {{"ancho": {width}, "alto": {height}}}
    }}
    """
    
    try:
        # Crear mensaje para GPT con imagen
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
        
        # Llamar a GPT
        response = llm.invoke([message])
        analysis = response.content
        
        print(f"✅ Análisis del Frame {current_frame} completado")
        
        state["analysis_result"] = analysis
        
        # Extraer víctimas del análisis
        try:
            # Buscar el JSON en la respuesta
            json_start = analysis.find('{')
            json_end = analysis.rfind('}') + 1
            if json_start != -1 and json_end != -1:
                json_str = analysis[json_start:json_end]
                analysis_data = json.loads(json_str)
                
                if "victimas_identificadas" in analysis_data:
                    state["victims_found"] = analysis_data["victimas_identificadas"]
                    print(f" Víctimas encontradas en este frame: {len(analysis_data['victimas_identificadas'])}")
                    
                    # Mostrar coordenadas de las víctimas
                    for i, victim in enumerate(analysis_data["victimas_identificadas"], 1):
                        coords = victim.get("coordenadas", {})
                        x = coords.get("x", "N/A")
                        y = coords.get("y", "N/A")
                        estado = victim.get("estado", "N/A")
                        print(f"   Víctima {i}: Estado={estado}, Posición=({x}, {y})")
        except Exception as e:
            print(f"⚠️ No se pudo extraer datos JSON del análisis: {e}")
        
    except Exception as e:
        print(f"❌ Error en análisis: {e}")
        state["analysis_result"] = f"Error: {e}"
    
    return state

def identificacionObstaculos(state: State) -> State:
    """Analiza el frame para identificar obstáculos específicamente"""
    frame_base64 = state["frame_base64"]
    current_frame = state["current_frame"]
    frame_data = state["frame_data"]
    
    print(f" Analizando Frame {current_frame} para identificar obstáculos...")
    
    # Obtener dimensiones de la imagen
    height, width = frame_data.shape[:2]
    
    # Prompt específico para identificación de obstáculos con coordenadas
    prompt = f"""
    Eres un sistema de análisis de video aéreo para rescate en desastres. 
    
    INFORMACIÓN DE LA IMAGEN:
    - Resolución: {width} x {height} píxeles
    - Ancho: {width} píxeles
    - Alto: {height} píxeles
    
    SISTEMA DE COORDENADAS:
    - SISTEMA DE PÍXELES (imagen): 
      * (0, 0) = esquina superior izquierda de la imagen
      * ({width}, {height}) = esquina inferior derecha de la imagen
      * Centro de imagen: ({width//2}, {height//2}) píxeles
    
    - SISTEMA DE COORDENADAS DEL MUNDO:
      * El mundo tiene 100x100 metros
      * Rango del mundo: X desde -50 a +50 metros, Y desde -50 a +50 metros
      * Centro del mundo (0, 0) corresponde al centro de la imagen ({width//2}, {height//2}) píxeles
      * Posición del rover (punto de referencia): (40, 40) metros en coordenadas del mundo
      * Conversión aproximada: 1 metro del mundo ≈ {width/100:.1f} píxeles de la imagen
      * El píxel ({width//2}, {height//2}) corresponde a (0, 0) en coordenadas del mundo
      * Esquina superior izquierda ({0}, {0}) corresponde aproximadamente a (-50, -50) metros
      * Esquina inferior derecha ({width}, {height}) corresponde aproximadamente a (+50, +50) metros
    
    ANALIZA ESTE FRAME ESPECÍFICAMENTE PARA IDENTIFICAR OBSTÁCULOS:
    
    - Los obstáculos son figuras CUADRADAS de color sólido que representan estructuras:
      * MARRÓN (#8B4513, RGB: 139, 69, 19): Edificios/estructuras
      * MAGENTA (#FF00FF, RGB: 255, 0, 255): Escombros
      * AMARILLO (#C8AA3C, RGB: 200, 170, 60): Árboles
      * GRIS OSCURO (#404040, RGB: 64, 64, 64): Vehículos
    
    - IMPORTANTE: El ROVER es un CUADRADO AZUL (#0000FF, RGB: 0, 0, 255) CON BORDE NEGRO - NO es un obstáculo, NO debe ser identificado como obstáculo
    - Las víctimas son círculos de colores:
      * ROJO (#FF0000, RGB: 255, 0, 0) = crítico
      * NARANJA (#FF6700, RGB: 255, 103, 0) = herido
      * VERDE OSCURO (#006400, RGB: 0, 100, 0) = seguro
      - NO son obstáculos
    - El fondo es marrón claro con cielo azul
    
    ⚠️ CRÍTICO: PRECISIÓN ES FUNDAMENTAL
    Tómate TODO el tiempo necesario para analizar la imagen cuidadosamente. 
    La precisión en las coordenadas es MÁS IMPORTANTE que la velocidad del análisis.
    
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
      - MARRÓN (#8B4513, RGB: 139, 69, 19) = edificio/estructura
      - MAGENTA (#FF00FF, RGB: 255, 0, 255) = escombro
      - AMARILLO (#C8AA3C, RGB: 200, 170, 60) = árbol
      - GRIS OSCURO (#404040, RGB: 64, 64, 64) = vehículo
    6. Si un cuadrado está parcialmente fuera del frame:
       - Estima el centro visible con la mayor precisión posible
       - Indica en las coordenadas el punto central estimado del cuadrado completo
    
    Responde en formato JSON:
    {{
        "obstaculos_identificados": [
            {{
                "id": 1,
                "coordenadas": {{"x": 150, "y": 200}},
                "tipo": "edificio/escombro/árbol/vehículo",
                "forma": "cuadrado",
                "color": "marrón/magenta/azul/gris",
                "tamaño": "pequeño/mediano/grande"
            }}
        ],
        "total_obstaculos": número_total,
        "resolucion_imagen": {{"ancho": {width}, "alto": {height}}}
    }}
    """
    
    try:
        # Crear mensaje para GPT con imagen
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
        
        # Llamar a GPT
        response = llm.invoke([message])
        analysis = response.content
        
        print(f"✅ Análisis de obstáculos del Frame {current_frame} completado")
        
        # Extraer obstáculos del análisis
        try:
            # Buscar el JSON en la respuesta
            json_start = analysis.find('{')
            json_end = analysis.rfind('}') + 1
            if json_start != -1 and json_end != -1:
                json_str = analysis[json_start:json_end]
                analysis_data = json.loads(json_str)
                
                if "obstaculos_identificados" in analysis_data:
                    state["obstacles_found"] = analysis_data["obstaculos_identificados"]
                    print(f" Obstáculos encontrados en este frame: {len(analysis_data['obstaculos_identificados'])}")
                    
                    # Mostrar coordenadas de los obstáculos
                    for i, obstacle in enumerate(analysis_data["obstaculos_identificados"], 1):
                        coords = obstacle.get("coordenadas", {})
                        x = coords.get("x", "N/A")
                        y = coords.get("y", "N/A")
                        tipo = obstacle.get("tipo", "N/A")
                        print(f"   Obstáculo {i}: Tipo={tipo}, Posición=({x}, {y})")
        except Exception as e:
            print(f"⚠️ No se pudo extraer datos JSON del análisis de obstáculos: {e}")
        
    except Exception as e:
        print(f"❌ Error en análisis de obstáculos: {e}")
    
    return state

def planificacionRuta(state: State) -> State:
    """Planifica UNA ruta óptima desde (10,10) que pase por TODAS las víctimas evitando obstáculos"""
    current_frame = state["current_frame"]
    
    # Solo planificar rutas en el último frame usando víctimas y obstáculos del frame actual
    if current_frame >= state["max_frames_to_process"]:
        victims = state["victims_found"]
        obstacles = state["obstacles_found"]
        
        print(f"️ Planificando ruta óptima desde (10,10) que pase por TODAS las {len(victims)} víctimas...")
        
        # Crear prompt para planificación de UNA ruta que pase por todas las víctimas
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
        
                OBJETIVO: Planificar UNA SOLA ruta óptima desde la posición inicial del rover que pase por TODAS las víctimas, evitando obstáculos.                 

        IDENTIFICACIÓN DEL ROVER:
        - El ROVER es un CUADRADO AZUL CON BORDE NEGRO visible en la imagen
        - La posición del rover en la imagen corresponde a la posición de INICIO de la ruta
        - El rover está ubicado en (40, 40) metros en coordenadas del mundo
        - IMPORTANTE: El rover NO es un obstáculo, es el vehículo que ejecutará la ruta

        SISTEMA DE COORDENADAS:
        - Las coordenadas que recibes están en PÍXELES de la imagen (donde (0,0) es esquina superior izquierda)                                                
        - SISTEMA DEL MUNDO: El mundo tiene 100x100 metros (rango: X desde -50 a +50, Y desde -50 a +50 metros)                                                 
        - Centro de imagen (128, 128) píxeles = centro del mundo (0, 0) metros 
        - Posición del rover (punto de referencia y punto de INICIO de la ruta): (40, 40) metros en coordenadas del mundo                                               
        - Conversión: Las coordenadas de píxeles se convertirán automáticamente al sistema del mundo                                                        
        - Esquina superior izquierda (0, 0) píxeles ≈ (-50, -50) metros      
        - Esquina inferior derecha (256, 256) píxeles ≈ (+50, +50) metros    

        INFORMACIÓN DEL ENTORNO:
        - Posición inicial del rover (punto de INICIO de la ruta): (40, 40) metros en coordenadas del mundo                                                                           
        - El rover es el cuadrado azul con borde negro que aparece en la imagen
        - La ruta DEBE comenzar desde la posición del rover (cuadrado azul con borde negro)
        - Resolución del mapa: 256 x 256 píxeles
        - Nota: Las coordenadas de las víctimas y obstáculos que ves están en píxeles y se convertirán al mundo
        
        VÍCTIMAS A VISITAR (TODAS):
        {chr(10).join(victims_data)}
        
        OBSTÁCULOS A EVITAR:
        {chr(10).join(obstacles_data)}
        
        CRITERIOS DE OPTIMIZACIÓN:
        1. La ruta DEBE pasar por TODAS las víctimas
        2. Priorizar el orden de visita según la prioridad de las víctimas (críticas primero)
        3. Minimizar distancia total recorrida
        4. Evitar obstáculos manteniendo distancia de seguridad de 5-10 metros (equivalente a ~12-25 píxeles)
        5. Calcular puntos de paso intermedios para evitar obstáculos
        6. La ruta debe usar coordenadas en PÍXELES (se convertirán automáticamente al sistema del mundo)
        
                INSTRUCCIONES:
        1. Localiza el rover (cuadrado azul con borde negro) en la imagen - esa es la posición de INICIO
        2. Crea UNA SOLA ruta que comience desde la posición del rover y visite TODAS las víctimas
        3. El primer punto de la ruta DEBE ser la posición del rover (cuadrado azul con borde negro)
        4. Ordena las víctimas por prioridad (críticas primero, luego heridas, atrapadas, seguras)                                                            
        5. Calcula la secuencia óptima de visita considerando distancia y prioridad                                                                            
        6. Incluye puntos de paso intermedios para evitar obstáculos
        7. Estima la distancia total y tiempo de la ruta completa
        
        Responde en formato JSON:
        {{
            "ruta_optimizada": {{
                "ruta_id": 1,
                "descripcion": "Ruta única que visita todas las víctimas en orden de prioridad",
                "orden_victimas": [2, 3, 4, 1],
                "puntos_ruta": [
                    {{"x": 26, "y": 26, "tipo": "inicio"}},
                    {{"x": 50, "y": 30, "tipo": "punto_paso"}},
                    {{"x": 150, "y": 200, "tipo": "victima", "victima_id": 2}},
                    {{"x": 200, "y": 180, "tipo": "punto_paso"}},
                    {{"x": 150, "y": 150, "tipo": "victima", "victima_id": 3}},
                    {{"x": 100, "y": 120, "tipo": "punto_paso"}},
                    {{"x": 80, "y": 100, "tipo": "victima", "victima_id": 4}},
                    {{"x": 60, "y": 80, "tipo": "victima", "victima_id": 1}}
                ],
                "distancia_total": 850.5,
                "tiempo_estimado": 28.4,
                "victimas_criticas_visitadas": 3,
                "victimas_totales_visitadas": 4
            }},
            "resumen": {{
                "total_victimas_cubiertas": 4,
                "distancia_total": 850.5,
                "tiempo_total_estimado": 28.4,
                "orden_prioridad": "críticas → heridas → atrapadas → seguras"
            }}
        }}
        """
        
        try:
            # Llamar a GPT para planificación de rutas
            message = {
                "role": "user",
                "content": prompt
            }
            
            response = llm.invoke([message])
            route_analysis = response.content
            
            print(f"✅ Planificación de ruta completada")
            
            # Extraer ruta del análisis
            try:
                # Buscar el JSON en la respuesta
                json_start = route_analysis.find('{')
                json_end = route_analysis.rfind('}') + 1
                if json_start != -1 and json_end != -1:
                    json_str = route_analysis[json_start:json_end]
                    route_data = json.loads(json_str)
                    
                    if "ruta_optimizada" in route_data:
                        ruta = route_data["ruta_optimizada"]
                        state["routes_planned"] = [ruta]  # Una sola ruta
                        
                        distancia = ruta.get("distancia_total", "N/A")
                        tiempo = ruta.get("tiempo_estimado", "N/A")
                        victimas_totales = ruta.get("victimas_totales_visitadas", "N/A")
                        victimas_criticas = ruta.get("victimas_criticas_visitadas", "N/A")
                        descripcion = ruta.get("descripcion", "Sin descripción")
                        
                        print(f"️ Ruta optimizada: {victimas_totales} víctimas, Distancia={distancia}, Tiempo={tiempo}s")
                        print(f"      Descripción: {descripcion}")
                        
                        # Mostrar orden de víctimas
                        orden_victimas = ruta.get("orden_victimas", [])
                        if orden_victimas:
                            print(f"      Orden de visita: {orden_victimas}")
                        
                        # Mostrar puntos de ruta
                        puntos_ruta = ruta.get("puntos_ruta", [])
                        if puntos_ruta:
                            print(f"      Puntos de ruta:")
                            for j, punto in enumerate(puntos_ruta, 1):
                                x = punto.get("x", "N/A")
                                y = punto.get("y", "N/A")
                                tipo = punto.get("tipo", "desconocido")
                                victima_id = punto.get("victima_id", "")
                                
                                if tipo == "victima" and victima_id:
                                    print(f"         {j}. ({x}, {y}) - {tipo} (ID: {victima_id})")
                                else:
                                    print(f"         {j}. ({x}, {y}) - {tipo}")
                        
                        # Mostrar resumen general
                        if "resumen" in route_data:
                            resumen = route_data["resumen"]
                            print(f" Resumen: {resumen.get('total_victimas_cubiertas', 0)} víctimas cubiertas, {resumen.get('orden_prioridad', 'N/A')}")
                            
            except Exception as e:
                print(f"⚠️ No se pudo extraer datos JSON de la planificación: {e}")
            
        except Exception as e:
            print(f"❌ Error en planificación de rutas: {e}")
    
    return state

def briefingMision(state: State) -> State:
    """Recopila toda la información y genera un mensaje de misión para el agente terrestre"""
    current_frame = state["current_frame"]
    
    # Solo generar el briefing en el último frame
    if current_frame >= state["max_frames_to_process"]:
        print(f"📋 Generando briefing de misión para agente terrestre...")
        
        victims = state["victims_found"]
        obstacles = state["obstacles_found"]
        routes = state["routes_planned"]
        
        # Crear el mensaje de misión estructurado
        mission_data = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "mission_id": f"MISSION_{int(time.time())}",
            "mission_type": "RESCATE_TERRESTRE",
            "priority": "ALTA",
            "zona_analizada": {
                "resolucion": "640x480",
                "coordenadas_limites": {
                    "norte": 0,
                    "sur": 480,
                    "este": 640,
                    "oeste": 0
                }
            },
            "victimas_identificadas": [],
            "obstaculos_identificados": [],
            "ruta_optimizada": {},
            "instrucciones_especificas": [],
            "equipamiento_recomendado": [],
            "riesgos_identificados": []
        }
        
        # Procesar víctimas
        for i, victim in enumerate(victims, 1):
            coords = victim.get("coordenadas", {})
            x = coords.get("x", 0)
            y = coords.get("y", 0)
            estado = victim.get("estado", "desconocido")
            prioridad = victim.get("prioridad", "media")
            color = victim.get("color", "desconocido")
            
            victim_data = {
                "id": i,
                "coordenadas": {"x": x, "y": y},
                "estado": estado,
                "prioridad": prioridad,
                "color_identificacion": color,
                "necesidades_medicas": _get_medical_needs(estado),
                "equipamiento_requerido": _get_required_equipment(estado)
            }
            mission_data["victimas_identificadas"].append(victim_data)
        
        # Procesar obstáculos
        for i, obstacle in enumerate(obstacles, 1):
            coords = obstacle.get("coordenadas", {})
            x = coords.get("x", 0)
            y = coords.get("y", 0)
            tipo = obstacle.get("tipo", "desconocido")
            forma = obstacle.get("forma", "desconocida")
            tamaño = obstacle.get("tamaño", "desconocido")
            
            obstacle_data = {
                "id": i,
                "coordenadas": {"x": x, "y": y},
                "tipo": tipo,
                "forma": forma,
                "tamaño": tamaño,
                "nivel_riesgo": _get_risk_level(tipo),
                "recomendaciones": _get_obstacle_recommendations(tipo)
            }
            mission_data["obstaculos_identificados"].append(obstacle_data)
        
        # Procesar ruta optimizada
        if routes and len(routes) > 0:
            ruta = routes[0]
            mission_data["ruta_optimizada"] = {
                "descripcion": ruta.get("descripcion", ""),
                "orden_victimas": ruta.get("orden_victimas", []),
                "puntos_ruta": ruta.get("puntos_ruta", []),
                "distancia_total": ruta.get("distancia_total", 0),
                "tiempo_estimado": ruta.get("tiempo_estimado", 0),
                "victimas_totales": ruta.get("victimas_totales_visitadas", 0),
                "victimas_criticas": ruta.get("victimas_criticas_visitadas", 0)
            }
        
        # Generar instrucciones específicas
        mission_data["instrucciones_especificas"] = _generate_specific_instructions(victims, obstacles, routes)
        
        # Generar recomendaciones de equipamiento
        mission_data["equipamiento_recomendado"] = _generate_equipment_recommendations(victims, obstacles)
        
        # Identificar riesgos
        mission_data["riesgos_identificados"] = _identify_risks(obstacles)
        
        # Convertir a mensaje estructurado
        mission_brief = _format_mission_brief(mission_data)
        state["mission_brief"] = mission_brief
        
        print(f"✅ Briefing de misión generado")
        print(f"📤 Mensaje preparado para agente terrestre")
        print("=" * 80)
        print("📋 BRIEFING DE MISIÓN PARA AGENTE TERRESTRE")
        print("=" * 80)
        print(mission_brief)
        print("=" * 80)
    
    return state

def cargadorMensaje(state: State) -> State:
    """Carga el mensaje del mission briefing en el archivo uav_to_ugv_message.json"""
    current_frame = state["current_frame"]
    
    # Solo cargar mensaje en el último frame
    if current_frame >= state["max_frames_to_process"]:
        mission_brief = state.get("mission_brief", "")
        
        if mission_brief:
            print(f"📤 UAV: Cargando mensaje en uav_to_ugv_message.json...")
            
            try:
                # Crear estructura del mensaje
                message_data = {
                    "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "from": "UAV",
                    "to": "UGV",
                    "message_type": "MISSION_BRIEFING",
                    "mission_brief": mission_brief,
                    "status": "READY_FOR_UGV",
                    "message_id": f"MSG_{int(time.time())}"
                }
                
                # Guardar mensaje en archivo JSON
                script_dir = os.path.dirname(os.path.abspath(__file__))
                message_path = os.path.join(script_dir, "..", "uav_to_ugv_message.json")
                with open(message_path, "w", encoding="utf-8") as f:
                    json.dump(message_data, f, indent=2, ensure_ascii=False)
                
                print(f"✅ UAV: Mensaje cargado exitosamente en uav_to_ugv_message.json")
                print(f"📄 UAV: Archivo listo para que el UGV lo lea")
                
                # Log de comunicación
                communication_log = state.get("communication_log", [])
                communication_log.append({
                    "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "type": "MESSAGE_LOADED",
                    "to": "UGV",
                    "file": "uav_to_ugv_message.json",
                    "status": "SUCCESS"
                })
                state["communication_log"] = communication_log
                
            except Exception as e:
                print(f"❌ UAV: Error al cargar mensaje: {e}")
                
                # Log de error
                communication_log = state.get("communication_log", [])
                communication_log.append({
                    "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "type": "MESSAGE_LOADED",
                    "to": "UGV",
                    "file": "uav_to_ugv_message.json",
                    "status": "ERROR",
                    "error": str(e)
                })
                state["communication_log"] = communication_log
        else:
            print(f"⚠️ UAV: No hay mensaje compilado para cargar")
    
    return state

def _get_medical_needs(estado):
    """Simplificado - solo retorna el estado"""
    return [estado]

def _get_required_equipment(estado):
    """Simplificado - solo retorna equipamiento básico"""
    return ["Basic medical kit"]

def _get_risk_level(tipo):
    """Simplificado - solo niveles básicos"""
    risk_map = {
        "edificio": "ALTO",
        "escombro": "ALTO", 
        "vehículo": "MEDIO",
        "árbol": "BAJO"
    }
    return risk_map.get(tipo, "MEDIO")

def _get_obstacle_recommendations(tipo):
    """Simplificado - solo recomendaciones básicas"""
    return ["Avoid area"]

def _generate_specific_instructions(victims, obstacles, routes):
    """Simplificado - solo instrucciones esenciales"""
    instructions = []
    
    # Solo instrucciones de ruta
    if routes and len(routes) > 0:
        ruta = routes[0]
        instructions.append(f"Follow route: {ruta.get('distancia_total', 0)}m, {ruta.get('tiempo_estimado', 0)}s")
        instructions.append(f"Visit order: {ruta.get('orden_victimas', [])}")
    
    return instructions

def _generate_equipment_recommendations(victims, obstacles):
    """Simplificado - solo equipamiento básico"""
    return ["Basic rescue equipment"]

def _identify_risks(obstacles):
    """Simplificado - solo riesgos básicos"""
    risks = []
    obstacle_types = [o.get("tipo") for o in obstacles]
    
    if "edificio" in obstacle_types:
        risks.append("Structural collapse risk")
    if "escombro" in obstacle_types:
        risks.append("Debris movement risk")
    if "vehículo" in obstacle_types:
        risks.append("Vehicle explosion risk")
    
    return risks

def _format_mission_brief(mission_data):
    """Formatea el briefing de misión en un mensaje legible"""
    brief = f"""
MISSION BRIEFING - RESCATE TERRESTRE
===================================
ID: {mission_data['mission_id']}
Timestamp: {mission_data['timestamp']}
Priority: {mission_data['priority']}

ZONE ANALYSIS
=============
Map Resolution: {mission_data['zona_analizada']['resolucion']}
Victims: {len(mission_data['victimas_identificadas'])}
Obstacles: {len(mission_data['obstaculos_identificados'])}

👥 VÍCTIMAS A RESCATAR
=====================
"""
    
    for victim in mission_data['victimas_identificadas']:
        coords = victim['coordenadas']
        brief += f"Victim {victim['id']}: Position=({coords['x']}, {coords['y']}), State={victim['estado']}, Priority={victim['prioridad']}\n"
    
    brief += f"""
🚧 OBSTÁCULOS IDENTIFICADOS
=========================
"""
    
    for obstacle in mission_data['obstaculos_identificados']:
        coords = obstacle['coordenadas']
        brief += f"Obstacle {obstacle['id']}: Position=({coords['x']}, {coords['y']}), Type={obstacle['tipo']}, Risk={obstacle['nivel_riesgo']}\n"
    
    if mission_data['ruta_optimizada']:
        ruta = mission_data['ruta_optimizada']
        brief += f"""
OPTIMIZED ROUTE
===============
Distance: {ruta.get('distancia_total', 0)}m
Time: {ruta.get('tiempo_estimado', 0)}s
Victims to visit: {ruta.get('victimas_totales', 0)}
Visit order: {ruta.get('orden_victimas', [])}

Puntos de ruta:
"""
        for i, punto in enumerate(ruta.get('puntos_ruta', []), 1):
            x = punto.get('x', 'N/A')
            y = punto.get('y', 'N/A')
            tipo = punto.get('tipo', 'desconocido')
            victima_id = punto.get('victima_id', '')
            
            if tipo == "victima" and victima_id:
                brief += f"  {i}. ({x}, {y}) - {tipo} (ID: {victima_id})\n"
            else:
                brief += f"  {i}. ({x}, {y}) - {tipo}\n"
    
    return brief

def should_continue(state: State) -> str:
    """Determina si continuar procesando más frames"""
    current_frame = state["current_frame"]
    max_frames = state["max_frames_to_process"]
    
    if current_frame >= max_frames:
        return "end"
    else:
        return "continue"

def encode_frame_to_base64(frame, frame_number):
    """Convierte un frame de OpenCV a base64 SIN conversión de colores"""
    
    # NO convertir colores - usar el frame tal como está (BGR)
    # Codificar directamente como PNG
    encode_params = [cv2.IMWRITE_PNG_COMPRESSION, 0]  # Sin compresión
    success, buffer = cv2.imencode('.png', frame, encode_params)
    
    if not success:
        print("❌ Error al codificar frame como PNG")
        return ""
    
    # Convertir a base64
    frame_base64 = base64.b64encode(buffer).decode('utf-8')
    
    return frame_base64

def create_uav_graph():
    """Crea y compila el grafo de trabajo del UAV"""
    # Crear grafo
    workflow = StateGraph(State)
    
    # Agregar nodos (sin patrullaje - se gestiona fuera del workflow)
    workflow.add_node("procesadorVideo", procesadorVideo)
    workflow.add_node("identificacionVictimas", identificacionVictimas)
    workflow.add_node("identificacionObstaculos", identificacionObstaculos)
    workflow.add_node("planificacionRuta", planificacionRuta)
    workflow.add_node("briefingMision", briefingMision)
    workflow.add_node("cargadorMensaje", cargadorMensaje)

    # Definir flujo: procesamiento de frames directo (patrullaje externo)
    workflow.set_entry_point("procesadorVideo")
    
    # Flujo de procesamiento de frames
    workflow.add_edge("procesadorVideo", "identificacionVictimas")
    workflow.add_edge("identificacionVictimas", "identificacionObstaculos")
    workflow.add_edge("identificacionObstaculos", "planificacionRuta")
    workflow.add_edge("planificacionRuta", "briefingMision")
    workflow.add_edge("briefingMision", "cargadorMensaje")
    
    # Finalizar después del primer procesamiento (no loop infinito)
    workflow.add_edge("cargadorMensaje", END)
    
    # Compilar y retornar
    return workflow.compile()

# Variable global para el grafo compilado (para langgraph.json)
g = create_uav_graph()

def main():
    print("🛸 SISTEMA UAV CON PATRULLAJE CIRCULAR Y ANÁLISIS DE VIDEO")
    print("=" * 80)
    
    # Configuración
    video_path = "uav_simulation.mp4"
    max_frames = 1  # Cambiar aquí para procesar más frames
    
    print(f"🎬 Iniciando sistema UAV: {video_path}")
    print(f"📋 Flujo de trabajo:")
    print(f"   1. Patrullaje circular del área (perímetro completo)")
    print(f"   2. Procesamiento de {max_frames} frame(s)")
    print(f"   3. Identificación de víctimas y obstáculos")
    print(f"   4. Planificación de ruta y generación de briefing")
    print("=" * 80)
    
    # Abrir video
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"❌ Error: No se pudo abrir el video {video_path}")
        return
    
    # Información del video
    fps = cap.get(cv2.CAP_PROP_FPS)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    duration = total_frames / fps
    
    # Estado inicial
    initial_state = {
        "video_path": video_path,
        "current_frame": 0,
        "total_frames": total_frames,
        "frame_data": None,
        "frame_base64": "",
        "analysis_result": "",
        "video_cap": cap,
        "max_frames_to_process": max_frames,
        "victims_found": [],
        "obstacles_found": [],
        "routes_planned": [],
        "mission_brief": "",
        "communication_log": [],  # Log de comunicaciones
        # Campos del controlador de movimiento
        "uav_position": (10, 10),
        "patrol_route": [],
        "patrol_index": 0,
        "patrol_complete": False,
        "movement_speed": 3.0
    }
    
    # Usar el grafo global compilado
    app = g
    
    try:
        result = app.invoke(
            initial_state,
            config={"recursion_limit": 999999}  # Límite muy alto para efectivamente ilimitado
        )
        
    except Exception as e:
        print(f"❌ Error durante el procesamiento: {e}")
    finally:
        cap.release()
        print("🔚 Procesamiento completado")

if __name__ == "__main__":
    main()
