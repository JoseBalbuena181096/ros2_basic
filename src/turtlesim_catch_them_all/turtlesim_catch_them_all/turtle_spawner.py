#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from my_robot_interfaces.msg import Turtle, TurtleArray
from my_robot_interfaces.srv import CatchTurtle
from functools import partial
import random
import math


class TurtleSpawnerNode(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        
        # Parámetros
        self.declare_parameter("spawn_frequency", 1.0)
        self.declare_parameter("turtle_name_prefix", "turtle")
        
        self.spawn_frequency_ = self.get_parameter(
            "spawn_frequency").get_parameter_value().double_value
        self.turtle_name_prefix_ = self.get_parameter(
            "turtle_name_prefix").get_parameter_value().string_value
        
        # Variables
        self.alive_turtles_ = []
        self.turtle_counter_ = 2  # Empezamos en 2 porque turtle1 ya existe
        
        # Publicador
        self.alive_turtles_publisher_ = self.create_publisher(
            TurtleArray, "/alive_turtles", 10)
        
        # Servidor de servicio
        self.catch_turtle_server_ = self.create_service(
            CatchTurtle, "/catch_turtle", self.callback_catch_turtle)
        
        # Timer para spawning
        self.spawn_timer_ = self.create_timer(
            1.0 / self.spawn_frequency_, self.spawn_new_turtle)
        
        self.get_logger().info("Turtle spawner has been started")
    
    def spawn_new_turtle(self):
        # Generar coordenadas aleatorias
        x = random.uniform(1.0, 10.0)
        y = random.uniform(1.0, 10.0)
        theta = random.uniform(0.0, 2 * math.pi)
        
        # Generar nombre
        name = f"{self.turtle_name_prefix_}{self.turtle_counter_}"
        self.turtle_counter_ += 1
        
        # Llamar al servicio de spawn
        self.call_spawn_server(name, x, y, theta)
    
    def call_spawn_server(self, name, x, y, theta):
        client = self.create_client(Spawn, "/spawn")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for /spawn service...")
        
        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name
        
        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_spawn, name=name, x=x, y=y, theta=theta))
    
    def callback_call_spawn(self, future, name, x, y, theta):
        try:
            response = future.result()
            self.get_logger().info(f"Spawned turtle: {response.name}")
            
            # Agregar la tortuga al array
            new_turtle = Turtle()
            new_turtle.name = response.name
            new_turtle.x = x
            new_turtle.y = y
            new_turtle.theta = theta
            self.alive_turtles_.append(new_turtle)
            
            # Publicar el array actualizado
            self.publish_alive_turtles()
        except Exception as e:
            self.get_logger().error(f"Spawn service call failed: {e}")
    
    def callback_catch_turtle(self, request, response):
        turtle_name = request.name
        self.get_logger().info(f"Catching turtle: {turtle_name}")
        
        # Llamar al servicio /kill
        self.call_kill_server(turtle_name)
        
        response.success = True
        return response
    
    def call_kill_server(self, name):
        client = self.create_client(Kill, "/kill")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for /kill service...")
        
        request = Kill.Request()
        request.name = name
        
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_kill, name=name))
    
    def callback_call_kill(self, future, name):
        try:
            future.result()
            self.get_logger().info(f"Killed turtle: {name}")
            
            # Remover la tortuga del array
            self.alive_turtles_ = [t for t in self.alive_turtles_ if t.name != name]
            
            # Publicar el array actualizado
            self.publish_alive_turtles()
        except Exception as e:
            self.get_logger().error(f"Kill service call failed: {e}")
    
    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles_
        self.alive_turtles_publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawnerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
