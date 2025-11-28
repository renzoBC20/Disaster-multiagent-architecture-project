#!/usr/bin/env python3
"""
Script de prueba para verificar la transformación de coordenadas.
"""

import sys
import os

# Agregar path al módulo multiagent
sys.path.insert(0, os.path.join(os.path.dirname(__file__)))

from multiagent.coordinate_transforms import CoordinateTransformer

def test_coordinate_transform():
    """Prueba la transformación de coordenadas."""
    
    # Crear transformador
    transformer = CoordinateTransformer(
        image_width=256,
        image_height=256,
        world_size=100.0  # 100 metros total (-50 a +50)
    )
    
    print("=" * 60)
    print("PRUEBA DE TRANSFORMACIÓN DE COORDENADAS")
    print("=" * 60)
    print()
    
    # Casos de prueba: pixel → mundo
    print("1. PRUEBA: PIXEL → MUNDO")
    print("-" * 60)
    
    test_cases_pixel_to_world = [
        ((0, 0), (-50, -50), "Esquina superior izquierda"),
        ((256, 256), (50, 50), "Esquina inferior derecha"),
        ((128, 128), (0, 0), "Centro de la imagen"),
        ((0, 256), (-50, 50), "Esquina inferior izquierda"),
        ((256, 0), (50, -50), "Esquina superior derecha"),
        ((64, 64), (-25, -25), "Cuarto superior izquierdo"),
        ((192, 192), (25, 25), "Cuarto inferior derecho"),
    ]
    
    all_passed = True
    for pixel, expected_world, description in test_cases_pixel_to_world:
        world = transformer.pixel_to_world(pixel[0], pixel[1])
        passed = abs(world[0] - expected_world[0]) < 0.01 and abs(world[1] - expected_world[1]) < 0.01
        status = "✅" if passed else "❌"
        
        if not passed:
            all_passed = False
        
        print(f"{status} {description}")
        print(f"   Pixel: ({pixel[0]}, {pixel[1]})")
        print(f"   Esperado mundo: ({expected_world[0]}, {expected_world[1]})")
        print(f"   Obtenido mundo: ({world[0]:.2f}, {world[1]:.2f})")
        print()
    
    # Casos de prueba: mundo → pixel
    print("2. PRUEBA: MUNDO → PIXEL (round-trip)")
    print("-" * 60)
    
    test_cases_world_to_pixel = [
        ((-50, -50), (0, 0), "Esquina superior izquierda"),
        ((50, 50), (256, 256), "Esquina inferior derecha"),
        ((0, 0), (128, 128), "Centro del mundo"),
        ((-50, 50), (0, 256), "Esquina inferior izquierda"),
        ((50, -50), (256, 0), "Esquina superior derecha"),
        ((-25, -25), (64, 64), "Cuarto superior izquierdo"),
        ((25, 25), (192, 192), "Cuarto inferior derecho"),
    ]
    
    for world, expected_pixel, description in test_cases_world_to_pixel:
        pixel = transformer.world_to_pixel(world[0], world[1])
        passed = abs(pixel[0] - expected_pixel[0]) <= 1 and abs(pixel[1] - expected_pixel[1]) <= 1
        status = "✅" if passed else "❌"
        
        if not passed:
            all_passed = False
        
        print(f"{status} {description}")
        print(f"   Mundo: ({world[0]}, {world[1]})")
        print(f"   Esperado pixel: ({expected_pixel[0]}, {expected_pixel[1]})")
        print(f"   Obtenido pixel: ({pixel[0]}, {pixel[1]})")
        print()
    
    # Prueba round-trip: pixel → mundo → pixel
    print("3. PRUEBA: ROUND-TRIP (pixel → mundo → pixel)")
    print("-" * 60)
    
    test_pixels = [
        (0, 0), (128, 128), (256, 256),
        (64, 64), (192, 192), (50, 150), (200, 100)
    ]
    
    for pixel in test_pixels:
        world = transformer.pixel_to_world(pixel[0], pixel[1])
        pixel_back = transformer.world_to_pixel(world[0], world[1])
        
        # Permitir error de 1 píxel (por redondeo)
        error_x = abs(pixel_back[0] - pixel[0])
        error_y = abs(pixel_back[1] - pixel[1])
        passed = error_x <= 1 and error_y <= 1
        status = "✅" if passed else "❌"
        
        if not passed:
            all_passed = False
        
        print(f"{status} Pixel original: ({pixel[0]}, {pixel[1]})")
        print(f"   → Mundo: ({world[0]:.2f}, {world[1]:.2f})")
        print(f"   → Pixel recuperado: ({pixel_back[0]}, {pixel_back[1]})")
        if not passed:
            print(f"   ⚠️ Error: ({error_x}, {error_y}) píxeles")
        print()
    
    # Prueba round-trip: mundo → pixel → mundo
    print("4. PRUEBA: ROUND-TRIP (mundo → pixel → mundo)")
    print("-" * 60)
    
    test_worlds = [
        (-50, -50), (0, 0), (50, 50),
        (-25, -25), (25, 25), (-10, 30), (40, -20)
    ]
    
    for world in test_worlds:
        pixel = transformer.world_to_pixel(world[0], world[1])
        world_back = transformer.pixel_to_world(pixel[0], pixel[1])
        
        # Permitir error de 0.1 metros
        error_x = abs(world_back[0] - world[0])
        error_y = abs(world_back[1] - world[1])
        passed = error_x < 0.1 and error_y < 0.1
        status = "✅" if passed else "❌"
        
        if not passed:
            all_passed = False
        
        print(f"{status} Mundo original: ({world[0]}, {world[1]})")
        print(f"   → Pixel: ({pixel[0]}, {pixel[1]})")
        print(f"   → Mundo recuperado: ({world_back[0]:.2f}, {world_back[1]:.2f})")
        if not passed:
            print(f"   ⚠️ Error: ({error_x:.2f}, {error_y:.2f}) metros")
        print()
    
    # Resumen
    print("=" * 60)
    if all_passed:
        print("✅ TODAS LAS PRUEBAS PASARON")
    else:
        print("❌ ALGUNAS PRUEBAS FALLARON")
    print("=" * 60)
    
    return all_passed

if __name__ == "__main__":
    success = test_coordinate_transform()
    sys.exit(0 if success else 1)

