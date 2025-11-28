#!/usr/bin/env python3
"""
Prueba manual de transformación de coordenadas - cálculo directo.
"""

# Parámetros
image_width = 256
image_height = 256
world_size = 100.0
world_min = -world_size / 2.0  # -50
world_max = world_size / 2.0   # +50

def pixel_to_world(pixel_x, pixel_y):
    """Transformación pixel → mundo."""
    norm_x = pixel_x / image_width
    norm_y = pixel_y / image_height
    world_x = world_min + norm_x * (world_max - world_min)
    world_y = world_min + norm_y * (world_max - world_min)
    return (world_x, world_y)

def world_to_pixel(world_x, world_y):
    """Transformación mundo → pixel."""
    norm_x = (world_x - world_min) / (world_max - world_min)
    norm_y = (world_y - world_min) / (world_max - world_min)
    pixel_x = int(norm_x * image_width)
    pixel_y = int(norm_y * image_height)
    # Asegurar límites
    pixel_x = max(0, min(image_width - 1, pixel_x))
    pixel_y = max(0, min(image_height - 1, pixel_y))
    return (pixel_x, pixel_y)

print("=" * 70)
print("PRUEBA DE TRANSFORMACIÓN DE COORDENADAS")
print("=" * 70)
print()

# Casos de prueba críticos
test_cases = [
    # (pixel, mundo_esperado, descripción)
    ((0, 0), (-50, -50), "Esquina superior izquierda"),
    ((256, 256), (50, 50), "Esquina inferior derecha"),
    ((128, 128), (0, 0), "Centro"),
    ((0, 256), (-50, 50), "Esquina inferior izquierda"),
    ((256, 0), (50, -50), "Esquina superior derecha"),
]

print("1. PRUEBA: PIXEL → MUNDO")
print("-" * 70)
all_passed = True

for pixel, expected, desc in test_cases:
    world = pixel_to_world(pixel[0], pixel[1])
    passed = abs(world[0] - expected[0]) < 0.01 and abs(world[1] - expected[1]) < 0.01
    status = "✅ PASS" if passed else "❌ FAIL"
    if not passed:
        all_passed = False
    
    print(f"{status} {desc}")
    print(f"   Pixel: ({pixel[0]}, {pixel[1]})")
    print(f"   Esperado: ({expected[0]}, {expected[1]})")
    print(f"   Obtenido: ({world[0]:.2f}, {world[1]:.2f})")
    print()

print("2. PRUEBA: MUNDO → PIXEL")
print("-" * 70)

world_test_cases = [
    ((-50, -50), (0, 0), "Esquina superior izquierda"),
    ((50, 50), (256, 256), "Esquina inferior derecha"),
    ((0, 0), (128, 128), "Centro"),
    ((-50, 50), (0, 256), "Esquina inferior izquierda"),
    ((50, -50), (256, 0), "Esquina superior derecha"),
]

for world, expected, desc in world_test_cases:
    pixel = world_to_pixel(world[0], world[1])
    passed = abs(pixel[0] - expected[0]) <= 1 and abs(pixel[1] - expected[1]) <= 1
    status = "✅ PASS" if passed else "❌ FAIL"
    if not passed:
        all_passed = False
    
    print(f"{status} {desc}")
    print(f"   Mundo: ({world[0]}, {world[1]})")
    print(f"   Esperado: ({expected[0]}, {expected[1]})")
    print(f"   Obtenido: ({pixel[0]}, {pixel[1]})")
    print()

print("3. PRUEBA: ROUND-TRIP (pixel → mundo → pixel)")
print("-" * 70)

test_pixels = [(0, 0), (128, 128), (256, 256), (64, 64), (192, 192)]
for pixel in test_pixels:
    world = pixel_to_world(pixel[0], pixel[1])
    pixel_back = world_to_pixel(world[0], world[1])
    error_x = abs(pixel_back[0] - pixel[0])
    error_y = abs(pixel_back[1] - pixel[1])
    passed = error_x <= 1 and error_y <= 1
    status = "✅ PASS" if passed else "❌ FAIL"
    if not passed:
        all_passed = False
    
    print(f"{status} Pixel: ({pixel[0]}, {pixel[1]}) → Mundo: ({world[0]:.2f}, {world[1]:.2f}) → Pixel: ({pixel_back[0]}, {pixel_back[1]})")
    if not passed:
        print(f"   ⚠️ Error: ({error_x}, {error_y}) píxeles")

print()
print("=" * 70)
if all_passed:
    print("✅ TODAS LAS PRUEBAS PASARON")
else:
    print("❌ ALGUNAS PRUEBAS FALLARON")
print("=" * 70)

