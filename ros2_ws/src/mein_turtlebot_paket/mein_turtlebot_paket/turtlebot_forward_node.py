#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time

class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__('color_detector_node')
        
        # Initialisiere Bridge für Konvertierung zwischen ROS und OpenCV
        self.bridge = CvBridge()
        
        # Erstelle Subscriber für Kamerabild
        self.image_subscription = self.create_subscription(
            Image,
            '/image_raw',  # Topic für ros2_v4l2_camera
            self.image_callback,
            10)
            
        # Debug-Info bei Initialisierung
        self.get_logger().info('Warte auf Kamerabild vom Topic "/image_raw"...')
        
        # Erstelle Publisher für Bewegungsbefehle
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
            
        # Status-Variablen
        self.target_detected = False
        self.is_turning = False
        self.turn_start_time = None
        self.state = "SEARCHING"  # SEARCHING, APPROACHING, STOPPING, TURNING
        
        # Parameter für die Farbdetektion (Rot in HSV)
        # Rot ist in HSV an den Rändern des H-Kanals, daher zwei Bereiche
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])
        
        # Schwellenwerte
        self.min_contour_area = 2000  # Minimale Fläche für Erkennung
        self.target_distance = 0.2  # Zieldistanz in Metern
        self.distance_tolerance = 0.05  # Toleranz in Metern
        
        self.get_logger().info('Color Detector Node wurde initialisiert')

    def image_callback(self, msg):
        try:
            # Konvertiere ROS-Bild zu OpenCV-Format
            # Explizites Konvertieren zu bgr8, falls die Kamera ein anderes Format sendet
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Debug-Info ausgeben
            self.get_logger().info(f'Bild empfangen: Format={msg.encoding}, Größe={msg.width}x{msg.height}')
            
            # Konvertiere Bild zu HSV für bessere Farberkennung
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Erstelle Maske für rote Farbe (zwei Bereiche)
            mask1 = cv2.inRange(hsv_image, self.lower_red1, self.upper_red1)
            mask2 = cv2.inRange(hsv_image, self.lower_red2, self.upper_red2)
            red_mask = cv2.bitwise_or(mask1, mask2)
            
            # Finde Konturen in der Maske
            contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Initialisiere Nachricht für Bewegungsbefehle
            twist_msg = Twist()
            
            # Wenn wir gerade dabei sind, uns zu drehen
            if self.state == "TURNING":
                current_time = time.time()
                # Vollständige Drehung (ca. 6 Sekunden bei 1.0 rad/s)
                if current_time - self.turn_start_time > 6.0:  
                    self.state = "SEARCHING"
                    twist_msg.angular.z = 0.0
                    self.get_logger().info('Drehung abgeschlossen, zurück zum Suchen')
                else:
                    twist_msg.angular.z = 1.0  # Weiterdrehen
            
            # Sonst nach rotem Ziel suchen und entsprechend handeln
            else:
                # Suche nach dem größten roten Objekt
                largest_contour = None
                largest_area = 0
                
                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area > largest_area and area > self.min_contour_area:
                        largest_area = area
                        largest_contour = contour
                
                if largest_contour is not None:
                    # Ziel gefunden
                    self.target_detected = True
                    
                    # Berechne Momentane (Schwerpunkt)
                    M = cv2.moments(largest_contour)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # Zeichne Kreis um Schwerpunkt zur Visualisierung
                        cv2.circle(cv_image, (cx, cy), 5, (0, 255, 0), -1)
                        
                        # Bildmitte bestimmen
                        height, width, _ = cv_image.shape
                        center_x = width // 2
                        
                        # Berechne horizontalen Fehler
                        error_x = cx - center_x
                        
                        # Schätze die Entfernung basierend auf der Größe des Objekts
                        # Je größer das Objekt, desto näher ist es
                        # Dies ist eine einfache Annäherung und kann kalibriert werden
                        normalized_area = largest_area / (width * height)
                        estimated_distance = 1.0 / (normalized_area * 10 + 0.1)  # Anpassen nach Bedarf
                        
                        self.get_logger().info(f'Rotes Objekt gefunden: Fläche={largest_area}, ' +
                                               f'Geschätzte Distanz={estimated_distance:.2f}m')
                        
                        if self.state == "SEARCHING" or self.state == "APPROACHING":
                            # Wenn wir das Ziel noch nicht erreicht haben
                            if estimated_distance > self.target_distance + self.distance_tolerance:
                                self.state = "APPROACHING"
                                # Ausrichten und vorwärts bewegen
                                twist_msg.angular.z = -float(error_x) / 500  # Proportionale Steuerung
                                twist_msg.linear.x = 0.2  # Vorwärts mit moderater Geschwindigkeit
                            else:
                                # Wir sind am Ziel
                                self.state = "STOPPING"
                                twist_msg.linear.x = 0.0
                                twist_msg.angular.z = 0.0
                                self.get_logger().info('Ziel erreicht!')
                                
                                
                else:
                    # Kein rotes Objekt gefunden
                    self.target_detected = False
                    if self.state == "SEARCHING":
                        # Langsam drehen, um nach Objekten zu suchen
                        twist_msg.angular.z = 0.5
            
            # Debug-Anzeige (nur aktivieren, wenn ein Display angeschlossen ist)
            # Für Headless-Betrieb können Sie diese Zeilen auskommentieren
            try:
                cv2.imshow("Camera View", cv_image)
                cv2.imshow("Red Mask", red_mask)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().warn(f'Kann Bild nicht anzeigen: {str(e)}')
            
            # Veröffentliche Bewegungsbefehl
            self.cmd_vel_publisher.publish(twist_msg)
            
        except Exception as e:
            self.get_logger().error(f'Fehler bei der Bildverarbeitung: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    color_detector_node = ColorDetectorNode()
    
    try:
        rclpy.spin(color_detector_node)
    except KeyboardInterrupt:
        pass
    
    # Stoppe den Roboter, bevor wir beenden
    stop_msg = Twist()
    color_detector_node.cmd_vel_publisher.publish(stop_msg)
    
    # Aufräumen
    cv2.destroyAllWindows()
    color_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()