# 🚀 INICIO RÁPIDO - Turtlesim Catch Them All

## ⚡ Ejecutar el Proyecto (1 comando)

```bash
cd /home/jose/ros2_ws && ./run_turtlesim_project.sh
```

## 📋 Lo que verás:
1. Se abre la ventana de Turtlesim
2. Aparece turtle1 (la tortuga maestra - azul)
3. Comienzan a aparecer tortugas nuevas cada segundo
4. turtle1 persigue y atrapa las tortugas automáticamente
5. Las tortugas desaparecen cuando son capturadas

## 🎮 Personalizar (Opcional)

Antes de ejecutar, edita los parámetros:
```bash
nano /home/jose/ros2_ws/src/my_robot_bringup/config/turtlesim_catch_them_all.yaml
```

### Parámetros disponibles:

```yaml
/turtle_controller:
  ros__parameters:
    catch_closest_turtle_first: true  # true = perseguir más cercana, false = perseguir primera

/turtle_spawner:
  ros__parameters:
    spawn_frequency: 1.0      # Tortugas por segundo (0.5 = lento, 2.0 = rápido)
    turtle_name_prefix: "turtle"  # Prefijo del nombre de las tortugas
```

## 🛑 Detener el Proyecto

Presiona `Ctrl + C` en la terminal

## 📊 Ver Información en Tiempo Real

### Terminal 2 - Ver tortugas vivas:
```bash
source /home/jose/ros2_ws/install/setup.bash
ros2 topic echo /alive_turtles
```

### Terminal 3 - Ver gráfico de nodos:
```bash
rqt_graph
```

## 🔧 Comandos Útiles

### Cambiar frecuencia de spawn (mientras corre):
```bash
ros2 param set /turtle_spawner spawn_frequency 2.0
```

### Cambiar estrategia de captura (mientras corre):
```bash
ros2 param set /turtle_controller catch_closest_turtle_first false
```

### Ver todos los nodos activos:
```bash
ros2 node list
```

### Ver todos los tópicos:
```bash
ros2 topic list
```

## 🐛 Solución de Problemas

### Si hay error al ejecutar:
```bash
cd /home/jose/ros2_ws
source install/setup.bash
ros2 launch my_robot_bringup turtlesim_catch_them_all.launch.py
```

### Si necesitas recompilar:
```bash
cd /home/jose/ros2_ws
colcon build --packages-select my_robot_interfaces turtlesim_catch_them_all my_robot_bringup
source install/setup.bash
```

## 📖 Documentación Completa

- **Resumen completo:** `/home/jose/ros2_ws/PROYECTO_RESUMEN.md`
- **README detallado:** `/home/jose/ros2_ws/src/turtlesim_catch_them_all/README.md`

---
¡Disfruta viendo cómo turtle1 atrapa a todas las tortugas! 🐢🎯
