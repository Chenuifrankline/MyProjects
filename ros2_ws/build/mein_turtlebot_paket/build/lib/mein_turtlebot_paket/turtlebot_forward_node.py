#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtlebotForwardNode(Node):
    def __init__(self):
        super().__init__('turtlebot_forward_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Turtlebot Forward Node gestartet')
        self.distance = 2.0  # Distanz in Metern
        self.linear_speed = 0.2  # m/s
        self.duration = self.distance / self.linear_speed  # Dauer in Sekunden
        
        # Verzögerung, um sicherzustellen, dass der Publisher bereit ist
        self.create_timer(1.0, self.move_forward_once)
        self.has_moved = False
        
    def move_forward_once(self):
        if not self.has_moved:
            self.has_moved = True
            self.move_forward()
        
    def move_forward(self):
        self.get_logger().info(f'Starte Bewegung: {self.distance} Meter vorwärts')
        start_time = time.time()
        
        # Schleife für die Bewegung
        while time.time() - start_time < self.duration:
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
        # Zusätzliche Stop-Befehle senden, um sicherzustellen, dass der Roboter anhält
        time.sleep(0.2)
        self.publisher.publish(stop_msg)
        
        self.get_logger().info('Bewegung beendet')
        self.get_logger().info(f'Gefahrene Strecke: {self.distance} Meter')

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotForwardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()