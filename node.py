import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__('color_detector_node')
        
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State machine
        self.state = "SEARCHING"  # SEARCHING, APPROACHING, STOPPED
        self.last_detection_time = self.get_clock().now()
        
        # Color detection parameters (Rot in HSV)
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])
        
        # Control parameters
        self.target_distance = 0.20  # 20 cm
        self.min_contour_area = 500
        self.spin_speed = 0.5  # rad/s
        self.approach_speed = 0.15  # m/s
        self.hysteresis = 0.05  # 5 cm Toleranz
        
        self.get_logger().info("Farbdetektor gestartet")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Rote Maske erstellen
            mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
            mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
            red_mask = cv2.bitwise_or(mask1, mask2)
            
            # Konturen finden
            contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            largest_contour = max(contours, key=cv2.contourArea, default=None)
            
            twist = Twist()
            
            if self.state == "SEARCHING":
                twist.angular.z = self.spin_speed  # Im Kreis drehen
                
                if largest_contour is not None and cv2.contourArea(largest_contour) > self.min_contour_area:
                    self.state = "APPROACHING"
                    self.get_logger().info("Rot erkannt! Starte Annäherung")
            
            elif self.state == "APPROACHING":
                if largest_contour is None or cv2.contourArea(largest_contour) < self.min_contour_area:
                    self.state = "SEARCHING"
                    self.get_logger().warn("Objekt verloren! Zurück zur Suche")
                    return
                
                # Distanzschätzung
                M = cv2.moments(largest_contour)
                cx = int(M["m10"]/M["m00"])
                cy = int(M["m01"]/M["m00"])
                
                # Einfache Distanzschätzung (Annahme: Kamera horizontal)
                pixel_height = cv2.boundingRect(largest_contour)[3]
                estimated_distance = (120 * 0.1) / pixel_height  # Kalibrierungswert anpassen
                
                # Regler für die Ausrichtung
                center_x = cv_image.shape[1] // 2
                angular_z = -0.005 * (cx - center_x)
                
                if estimated_distance > self.target_distance + self.hysteresis:
                    twist.linear.x = self.approach_speed
                    twist.angular.z = angular_z
                elif estimated_distance < self.target_distance - self.hysteresis:
                    twist.linear.x = -0.05  # Leicht zurückfahren
                else:
                    self.state = "STOPPED"
                    self.get_logger().info("Ziel erreicht! Halte an")
            
            elif self.state == "STOPPED":
                pass  # Keine Bewegung
                
            self.cmd_vel_pub.publish(twist)
            
            # Debug-Ansicht
            cv2.drawContours(cv_image, [largest_contour], -1, (0,255,0), 2) if largest_contour else None
            cv2.imshow("Kamera", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Fehler: {str(e)}")
            self.state = "SEARCHING"

def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Cleanup
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()