# 🔧 Cambios Realizados

## ✅ Modificación de Posiciones Iniciales

### Cambio Realizado

Se modificó el archivo `scenarios/default.yaml` para cambiar las posiciones iniciales de los robots:

**Antes:**
```yaml
drone: x: 0.0, y: 0.0
rover: x: -5.0, y: 0.0
```

**Después:**
```yaml
drone: x: -40.0, y: -40.0
rover: x: -40.0, y: -40.0
```

### Notas

- Los robots inician en la misma posición (esquina suroeste)
- Para aplicar cambios, recompila:
  ```cmd
  colcon build --packages-select microsim
  call install\setup.bat
  ```
