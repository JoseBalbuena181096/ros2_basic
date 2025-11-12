#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import Turtle, TurtleArray
from my_robot_interfaces.srv import CatchTurtle
from functools import partial
import math


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        
        # Parámetros
        self.declare_parameter("catch_closest_turtle_first", True)
        self.catch_closest_turtle_first_ = self.get_parameter(
            "catch_closest_turtle_first").get_parameter_value().bool_value
        
        # Variables
        self.pose_ = None
        self.target_turtle_ = None
        self.alive_turtles_ = []
        
        # Suscriptores
        self.pose_subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.callback_turtle_pose, 10)
        self.alive_turtles_subscriber_ = self.create_subscription(
            TurtleArray, "/alive_turtles", self.callback_alive_turtles, 10)
        
        # Publicadores
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        
        # Timer para el control loop
        self.control_timer_ = self.create_timer(0.01, self.control_loop)
        
        self.get_logger().info("Turtle controller has been started")
    
    def callback_turtle_pose(self, msg):
        self.pose_ = msg
    
    def callback_alive_turtles(self, msg):
        self.alive_turtles_ = msg.turtles
        
        if len(self.alive_turtles_) > 0:
            if self.target_turtle_ is None or self.target_turtle_.name not in [t.name for t in self.alive_turtles_]:
                if self.catch_closest_turtle_first_:
                    self.target_turtle_ = self.get_closest_turtle()
                else:
                    self.target_turtle_ = self.alive_turtles_[0]
        else:
            self.target_turtle_ = None
    
    def get_closest_turtle(self):
        if self.pose_ is None or len(self.alive_turtles_) == 0:
            return None
        
        closest_turtle = None
        min_distance = float('inf')
        
        for turtle in self.alive_turtles_:
            dist = math.sqrt((turtle.x - self.pose_.x)**2 + (turtle.y - self.pose_.y)**2)
            if dist < min_distance:
                min_distance = dist
                closest_turtle = turtle
        
        return closest_turtle
    
    def control_loop(self):
        if self.pose_ is None or self.target_turtle_ is None:
            return
        
        # Calcular distancia al objetivo
        dist_x = self.target_turtle_.x - self.pose_.x
        dist_y = self.target_turtle_.y - self.pose_.y
        distance = math.sqrt(dist_x**2 + dist_y**2)
        
        msg = Twist()
        
        # Si estamos muy cerca del objetivo, detener y llamar al servicio
        if distance < 0.5:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_vel_publisher_.publish(msg)
            self.call_catch_turtle_server(self.target_turtle_.name)
            self.target_turtle_ = None
            return
        
        # Control P para velocidad lineal
        msg.linear.x = 2.0 * distance
        
        # Control P para velocidad angular
        goal_theta = math.atan2(dist_y, dist_x)
        diff = goal_theta - self.pose_.theta
        
        # Normalizar el ángulo entre -pi y pi
        if diff > math.pi:
            diff -= 2 * math.pi
        elif diff < -math.pi:
            diff += 2 * math.pi
        
        msg.angular.z = 6.0 * diff
        
        self.cmd_vel_publisher_.publish(msg)
    
    def call_catch_turtle_server(self, turtle_name):
        client = self.create_client(CatchTurtle, "/catch_turtle")
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for /catch_turtle service...")
        
        request = CatchTurtle.Request()
        request.name = turtle_name
        
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_catch_turtle, turtle_name=turtle_name))
    
    def callback_call_catch_turtle(self, future, turtle_name):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Turtle {turtle_name} has been caught!")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
