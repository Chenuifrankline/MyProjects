#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtlebotForwardNode(Node):
    def __init__(self):
        super().__init__('turtlebot_forward_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.distance = 2.0  # Distanz in Metern
        self.linear_speed = 0.2  # m/s
        self.duration = self.distance / self.linear_speed  # Dauer in Sekunden
        self.start_time = None
        self.is_moving = False
        self.get_logger().info('Turtlebot Forward Node gestartet')
        
    def timer_callback(self):
        if not self.is_moving:
            self.move_forward()
            
    def move_forward(self):
        self.get_logger().info(f'Starte Bewegung: {self.distance} Meter vorwärts')
        self.is_moving = True
        self.start_time = time.time()
        
        # Schleife für die Bewegung
        while time.time() - self.start_time < self.duration:
            msg = Twist()
            msg.linear.x = self.linear_speed
            msg.angular.z = 0.0
            self.publisher.publish(msg)
            time.sleep(0.1)  # kleine Pause zwischen den Befehlen
            
        # Stoppen des Roboters
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher.publish(stop_msg)
        self.get_logger().info('Bewegung beendet')
        self.is_moving = False
        # Optional: Node beenden nach der Aufgabe
        # rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotForwardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()