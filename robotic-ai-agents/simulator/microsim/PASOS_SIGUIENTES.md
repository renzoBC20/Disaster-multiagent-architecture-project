# 📋 Pasos Siguientes

## ✅ Corrección Aplicada

He corregido el código para que busque el archivo `.env` en la ubicación correcta.

---

## 🔧 Pasos Para Continuar

### 1️⃣ Verificar Configuración

Ejecuta:
```cmd
cd "D:\Proyectos PFG\robotic-ai-agents\simulator\microsim"
VERIFICAR_CONFIGURACION.bat
```

Este script verificará:
- ✅ Si existe el archivo `.env`
- ✅ Si las dependencias están instaladas
- ✅ Si la API key está configurada correctamente

---

### 2️⃣ Instalar Dependencias (Si Faltan)

Si el paso 1 detecta dependencias faltantes, ejecuta:
```cmd
INSTALAR_DEPENDENCIAS.bat
```

---

### 3️⃣ Configurar API Key (Si Falta)

Si tu API key no está configurada, abre el archivo:
```
D:\Proyectos PFG\MultiAgent\.env
```

Y cambia:
```env
OPENAI_API_KEY=tu_api_key_aqui
```

Por:
```env
OPENAI_API_KEY=sk-proj-tu_api_key_real_aqui
```

---

### 4️⃣ Probar Sistema

Una vez que todo esté configurado, ejecuta en 4 terminales CMD:

**Terminal 1:**
```cmd
INICIAR.bat
```

**Terminal 2:**
```cmd
INICIAR_VISUALIZACION.bat
```

**Terminal 3:**
```cmd
INICIAR_LANGGRAPH.bat
```

**Terminal 4:**
```cmd
INICIAR_UGV.bat
```

---

## 🎯 Versiones Disponibles

### Versión Simplificada (Sin LangGraph Workflow)
- Más simple de depurar
- Usa directamente GPT para análisis
- **Script:** `INICIAR_CONTROLADOR.bat`

### Versión Completa (Con LangGraph)
- Respeta la arquitectura multi-agente original
- Usa StateGraph y workflows
- **Script:** `INICIAR_LANGGRAPH.bat`

**Recomendación:** Empieza con la versión simplificada para verificar que todo funciona, luego prueba LangGraph.

---

## 🆘 Troubleshooting

### Error: "No module named 'langgraph'"
**Solución:** Ejecuta `INSTALAR_DEPENDENCIAS.bat`

### Error: "api_key client option must be set"
**Solución:** Verifica que tu `.env` esté correctamente configurado

### Error: "cannot connect to simulator"
**Solución:** Asegúrate de ejecutar `INICIAR.bat` primero

---

**¿Preguntas?** Ejecuta `VERIFICAR_CONFIGURACION.bat` primero 🚀

