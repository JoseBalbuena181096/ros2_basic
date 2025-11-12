#!/bin/bash

# Script de testing para el proyecto Turtlesim Catch Them All
# Verifica que todo esté correctamente configurado antes de ejecutar

echo "╔════════════════════════════════════════════════════════════╗"
echo "║  🧪 Test del Proyecto Turtlesim Catch Them All           ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Colores
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Función para verificar
check() {
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓${NC} $1"
        return 0
    else
        echo -e "${RED}✗${NC} $1"
        return 1
    fi
}

# Source del workspace
echo "📦 Verificando workspace..."
source /home/jose/ros2_ws/install/setup.bash
check "Workspace sourced"

echo ""
echo "🔍 Verificando paquetes..."

# Verificar paquete turtlesim_catch_them_all
ros2 pkg list | grep -q "turtlesim_catch_them_all"
check "Paquete turtlesim_catch_them_all existe"

# Verificar ejecutables
ros2 pkg executables turtlesim_catch_them_all | grep -q "turtle_controller"
check "Ejecutable turtle_controller encontrado"

ros2 pkg executables turtlesim_catch_them_all | grep -q "turtle_spawner"
check "Ejecutable turtle_spawner encontrado"

echo ""
echo "📝 Verificando interfaces..."

# Verificar mensajes
ros2 interface list | grep -q "my_robot_interfaces/msg/Turtle"
check "Mensaje Turtle.msg encontrado"

ros2 interface list | grep -q "my_robot_interfaces/msg/TurtleArray"
check "Mensaje TurtleArray.msg encontrado"

# Verificar servicios
ros2 interface list | grep -q "my_robot_interfaces/srv/CatchTurtle"
check "Servicio CatchTurtle.srv encontrado"

echo ""
echo "🚀 Verificando launch files..."

# Verificar launch file
if [ -f "/home/jose/ros2_ws/src/my_robot_bringup/launch/turtlesim_catch_them_all.launch.py" ]; then
    echo -e "${GREEN}✓${NC} Launch file encontrado"
else
    echo -e "${RED}✗${NC} Launch file no encontrado"
fi

# Verificar archivo de configuración
if [ -f "/home/jose/ros2_ws/src/my_robot_bringup/config/turtlesim_catch_them_all.yaml" ]; then
    echo -e "${GREEN}✓${NC} Archivo de configuración encontrado"
else
    echo -e "${RED}✗${NC} Archivo de configuración no encontrado"
fi

echo ""
echo "📄 Verificando código fuente..."

# Verificar archivos Python
if [ -f "/home/jose/ros2_ws/src/turtlesim_catch_them_all/turtlesim_catch_them_all/turtle_controller.py" ]; then
    echo -e "${GREEN}✓${NC} turtle_controller.py existe"
else
    echo -e "${RED}✗${NC} turtle_controller.py no encontrado"
fi

if [ -f "/home/jose/ros2_ws/src/turtlesim_catch_them_all/turtlesim_catch_them_all/turtle_spawner.py" ]; then
    echo -e "${GREEN}✓${NC} turtle_spawner.py existe"
else
    echo -e "${RED}✗${NC} turtle_spawner.py no encontrado"
fi

echo ""
echo "🔧 Verificando paquete turtlesim..."
ros2 pkg list | grep -q "turtlesim"
check "Paquete turtlesim disponible"

echo ""
echo "════════════════════════════════════════════════════════════"
echo -e "${GREEN}✅ Verificación completada!${NC}"
echo ""
echo "Para ejecutar el proyecto:"
echo -e "${YELLOW}./run_turtlesim_project.sh${NC}"
echo ""
echo "O manualmente:"
echo -e "${YELLOW}ros2 launch my_robot_bringup turtlesim_catch_them_all.launch.py${NC}"
echo "════════════════════════════════════════════════════════════"
