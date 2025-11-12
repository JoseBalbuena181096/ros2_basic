# Turtlesim "Catch Them All" - Proyecto ROS2 Jazzy

Este proyecto implementa un sistema completo en ROS2 donde una tortuga maestra (turtle1) persigue y "atrapa" tortugas que aparecen aleatoriamente en la ventana de turtlesim.

## Estructura del Proyecto

### Paquetes Creados/Modificados:

1. **turtlesim_catch_them_all**: Paquete principal con los nodos personalizados
   - `turtle_controller`: Controla la tortuga maestra para perseguir objetivos
   - `turtle_spawner`: Genera tortugas aleatorias y gestiona su ciclo de vida

2. **my_robot_interfaces**: Interfaces personalizadas
   - `Turtle.msg`: Representa una tortuga (nombre, posición x, y, theta)
   - `TurtleArray.msg`: Array de tortugas vivas
   - `CatchTurtle.srv`: Servicio para notificar captura de una tortuga

3. **my_robot_bringup**: Archivos de lanzamiento y configuración
   - `turtlesim_catch_them_all.launch.py`: Launch file principal
   - `turtlesim_catch_them_all.yaml`: Parámetros configurables

## Características Implementadas

### Nodo turtle_controller:
- ✅ Control proporcional (P) para movimiento suave
- ✅ Suscripción a `/turtle1/pose` para obtener posición actual
- ✅ Publicación en `/turtle1/cmd_vel` para comandos de velocidad
- ✅ Suscripción a `/alive_turtles` para obtener objetivos
- ✅ Cliente del servicio `/catch_turtle` para notificar capturas
- ✅ Selección de tortuga más cercana (configurable)

### Nodo turtle_spawner:
- ✅ Generación automática de tortugas en posiciones aleatorias
- ✅ Publicación del array de tortugas vivas en `/alive_turtles`
- ✅ Servidor del servicio `/catch_turtle`
- ✅ Llamada al servicio `/kill` para eliminar tortugas capturadas
- ✅ Frecuencia de spawn configurable

### Parámetros:
- `catch_closest_turtle_first` (turtle_controller): Si es true, persigue la tortuga más cercana
- `spawn_frequency` (turtle_spawner): Frecuencia de generación de tortugas (Hz)
- `turtle_name_prefix` (turtle_spawner): Prefijo para nombres de tortugas

## Cómo Compilar

```bash
cd /home/jose/ros2_ws
colcon build
source install/setup.bash
```

## Cómo Ejecutar

### Opción 1: Usando el Launch File (Recomendado)
```bash
source /home/jose/ros2_ws/install/setup.bash
ros2 launch my_robot_bringup turtlesim_catch_them_all.launch.py
```

### Opción 2: Nodos Individuales
Terminal 1 - Turtlesim:
```bash
source /home/jose/ros2_ws/install/setup.bash
ros2 run turtlesim turtlesim_node
```

Terminal 2 - Turtle Spawner:
```bash
source /home/jose/ros2_ws/install/setup.bash
ros2 run turtlesim_catch_them_all turtle_spawner
```

Terminal 3 - Turtle Controller:
```bash
source /home/jose/ros2_ws/install/setup.bash
ros2 run turtlesim_catch_them_all turtle_controller
```

## Modificar Parámetros

Edita el archivo: `/home/jose/ros2_ws/src/my_robot_bringup/config/turtlesim_catch_them_all.yaml`

Ejemplo:
```yaml
/turtle_controller:
  ros__parameters:
    catch_closest_turtle_first: false  # Perseguir primera tortuga en el array

/turtle_spawner:
  ros__parameters:
    spawn_frequency: 2.0  # Generar 2 tortugas por segundo
    turtle_name_prefix: "target"  # Nombrar tortugas como target1, target2, etc.
```

## Comandos Útiles

### Ver tópicos activos:
```bash
ros2 topic list
```

### Ver info de tortugas vivas:
```bash
ros2 topic echo /alive_turtles
```

### Ver gráfico de nodos:
```bash
rqt_graph
```

### Cambiar parámetros en tiempo real:
```bash
# Cambiar frecuencia de spawn
ros2 param set /turtle_spawner spawn_frequency 0.5

# Cambiar estrategia de captura
ros2 param set /turtle_controller catch_closest_turtle_first false
```

## Arquitectura del Sistema

```
┌─────────────────┐
│  turtlesim_node │
│                 │
│  - /spawn srv   │
│  - /kill srv    │
│  - /turtle1/pose│
│  - /turtle1/cmd │
└────────┬────────┘
         │
         │ Services & Topics
         │
    ┌────┴────────────────────────┐
    │                             │
┌───▼──────────┐         ┌────────▼────────┐
│turtle_spawner│         │turtle_controller│
│              │         │                 │
│- Spawn @1Hz  │◄────────┤- P Controller   │
│- /alive_turtles Pub    │- Subscribe pose │
│- /catch_turtle Srv     │- Publish cmd_vel│
│              │         │- Call /catch    │
└──────────────┘         └─────────────────┘
```

## Detalles Técnicos

### Control Proporcional:
- Velocidad lineal: `v = 2.0 * distance`
- Velocidad angular: `ω = 6.0 * angle_error`
- Umbral de captura: 0.5 unidades

### Spawn Aleatorio:
- Rango X: [1.0, 10.0]
- Rango Y: [1.0, 10.0]
- Theta: [0, 2π]

## Solución de Problemas

### Si las tortugas no aparecen:
- Verifica que turtlesim_node esté corriendo
- Revisa los logs: `ros2 node info /turtle_spawner`

### Si la tortuga maestra no se mueve:
- Verifica que haya tortugas para perseguir
- Revisa: `ros2 topic echo /alive_turtles`

### Si hay errores de compilación:
```bash
cd /home/jose/ros2_ws
rm -rf build/ install/ log/
colcon build
```

## Autor
José Balbuena - Proyecto educativo ROS2 Jazzy

## Licencia
MIT
