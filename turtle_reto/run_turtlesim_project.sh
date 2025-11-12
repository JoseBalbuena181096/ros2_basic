#!/bin/bash

# Script para ejecutar el proyecto Turtlesim Catch Them All
# Autor: José Balbuena

echo "=========================================="
echo "   Turtlesim Catch Them All - ROS2 Jazzy"
echo "=========================================="
echo ""

# Source el workspace
source /home/jose/ros2_ws/install/setup.bash

echo "Iniciando el proyecto..."
echo "Presiona Ctrl+C para detener"
echo ""

# Ejecutar el launch file
ros2 launch my_robot_bringup turtlesim_catch_them_all.launch.py
