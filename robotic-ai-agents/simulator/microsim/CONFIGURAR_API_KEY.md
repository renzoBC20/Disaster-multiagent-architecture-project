# 🔑 Configurar API Key de OpenAI

## 📍 Ubicación del archivo

El archivo `.env` está en:
```
D:\Proyectos PFG\MultiAgent\.env
```

---

## 🔧 Configuración

### Paso 1: Obtener tu API Key

1. Ve a: https://platform.openai.com/api-keys
2. Inicia sesión (o crea una cuenta)
3. Haz clic en "Create new secret key"
4. Copia la key (solo se muestra una vez)

### Paso 2: Editar el archivo .env

1. Abre el archivo: `D:\Proyectos PFG\MultiAgent\.env`
2. Busca la línea:
   ```env
   OPENAI_API_KEY=tu_api_key_aqui
   ```
3. Reemplaza `tu_api_key_aqui` con tu API key real:
   ```env
   OPENAI_API_KEY=sk-proj-xxxxxxxxxxxxxxxxxxxxx
   ```
4. Guarda el archivo

---

## ⚠️ Importante

### Seguridad
- **NUNCA** subas el archivo `.env` a GitHub
- El archivo `.gitignore` ya incluye `.env` automáticamente
- No compartas tu API key públicamente

### Verificación
Para verificar que funciona, ejecuta en CMD:

```cmd
cd "D:\Proyectos PFG\robotic-ai-agents\simulator\microsim"
call "%LOCALAPPDATA%\miniforge3\condabin\conda.bat" activate ros2_humble
python -c "from dotenv import load_dotenv; import os; load_dotenv('../../MultiAgent/.env'); print('API Key:', os.getenv('OPENAI_API_KEY')[:10] + '...' if os.getenv('OPENAI_API_KEY') else 'NO CONFIGURADA')"
```

Deberías ver algo como: `API Key: sk-proj-ab...`

---

## 🎯 Alternativa: Variable de Entorno del Sistema

Si prefieres no usar el archivo `.env`, puedes configurar la variable de entorno del sistema:

### Windows CMD:
```cmd
setx OPENAI_API_KEY "sk-proj-xxxxxxxxxxxxxxxxxxxxx"
```

### Windows PowerShell:
```powershell
[Environment]::SetEnvironmentVariable("OPENAI_API_KEY", "sk-proj-xxxxxxxxxxxxxxxxxxxxx", "User")
```

Luego **reinicia** la terminal para que tome efecto.

---

## 📊 Costos de OpenAI

### Modelo: `gpt-5-mini` (Configurado actualmente)
- Precio aproximado: $0.15 por 1M tokens de entrada
- El análisis de imágenes consume tokens según la resolución

### Estimación de uso:
- Análisis de víctimas: ~2-5 tokens por frame
- Análisis de obstáculos: ~2-5 tokens por frame  
- Planificación de ruta: ~50-100 tokens por ruta
- **Total por misión completa**: ~500-1000 tokens ($0.0001 - $0.0002)

### Recomendaciones:
- Configura un límite de gasto en tu cuenta OpenAI
- Comienza con análisis menos frecuentes (cada 10-20 frames)
- Usa el modo básico sin IA para pruebas iniciales

---

## ✅ Verificar que funciona

Después de configurar, ejecuta el controlador UAV:

```cmd
cd "D:\Proyectos PFG\robotic-ai-agents\simulator\microsim"
call "%LOCALAPPDATA%\miniforge3\condabin\conda.bat" activate ros2_humble
call install\setup.bat
python scripts\uav_ai_controller.py
```

Deberías ver:
```
[INFO] [uav_ai_controller]: 🚁 UAV AI Controller inicializado!
[INFO] [uav_ai_controller]:    IA activada: True
[INFO] [uav_ai_controller]: 🔍 Iniciando análisis de IA...
```

Si ves `IA activada: False`, verifica que el archivo `.env` tenga la API key correcta.

---

## 🐛 Problemas Comunes

### "llm is None" o "IA activada: False"
- Verifica que el archivo `.env` tenga la línea correcta
- Verifica que no haya espacios alrededor de `=`
- Verifica que la API key empiece con `sk-`

### "No module named 'dotenv'"
```cmd
pip install python-dotenv
```

### "API Key inválida"
- Verifica que copiaste la key completa
- Verifica que no tenga caracteres extra
- Prueba obtener una nueva key desde OpenAI

---

## 📚 Más información

- [OpenAI API Documentation](https://platform.openai.com/docs)
- [python-dotenv Documentation](https://pypi.org/project/python-dotenv/)

