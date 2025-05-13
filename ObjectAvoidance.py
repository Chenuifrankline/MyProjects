#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import time

class ColorDetectorNode(Node):
    def _init_(self):
        super()._init_('color_detector_node')
        
        # Initialisiere Bridge für Konvertierung zwischen ROS und OpenCV
        self.bridge = CvBridge()
        
        # Erstelle Subscriber für Kamerabild
        self.image_subscription = self.create_subscription(
            Image,
            '/image_raw',  # Topic für ros2_v4l2_camera
            self.image_callback,
            10)
            
        # Erstelle Subscriber für Laser/Entfernungsmesser
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Standard-Topic für Laserscanner
            self.scan_callback,
            10)
            
        # Debug-Info bei Initialisierung
        self.get_logger().info('Warte auf Kamerabild vom Topic "/image_raw" und Scan-Daten vom Topic "/scan"...')
        
        # Erstelle Publisher für Bewegungsbefehle
        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
            
        # Status-Variablen
        self.target_detected = False
        self.is_turning = False
        self.turn_start_time = None
        self.state = "SEARCHING"  # SEARCHING, APPROACHING, STOPPING, TURNING, AVOIDING
        self.obstacle_detected = False
        self.obstacle_direction = None  # 'links' oder 'rechts'
        self.avoidance_start_time = None
        self.last_target_position = None  # (cx, cy) des letzten gesehenen Zielobjekts
        
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
        self.obstacle_distance_threshold = 0.5  # Schwelle für Hinderniserkennung in Metern
        
        # Speichern der Laserscan-Daten
        self.laser_data = None
        
        # Timer für den Zustandsautomaten
        self.create_timer(0.1, self.state_machine)
        
        self.get_logger().info('Color Detector Node wurde initialisiert')

    def scan_callback(self, msg):
        """Callback für Laserscan-Daten"""
        self.laser_data = msg
        # Prüfe auf Hindernisse im Vordergrund
        self.check_obstacles(msg)
        
    def check_obstacles(self, scan_msg):
        """Überprüft die Laserscan-Daten auf Hindernisse"""
        if scan_msg is None:
            return
            
        # Definiere Sichtbereiche (vorne, links vorne, rechts vorne)
        front_indices = range(len(scan_msg.ranges) // 2 - 30, len(scan_msg.ranges) // 2 + 30)
        left_front_indices = range(len(scan_msg.ranges) // 2 + 31, len(scan_msg.ranges) // 2 + 90)
        right_front_indices = range(len(scan_msg.ranges) // 2 - 90, len(scan_msg.ranges) // 2 - 31)
        
        # Filtere ungültige Messwerte
        front_valid = [r for r in [scan_msg.ranges[i] for i in front_indices] 
                      if r >= scan_msg.range_min and r <= scan_msg.range_max]
        left_valid = [r for r in [scan_msg.ranges[i] for i in left_front_indices] 
                     if r >= scan_msg.range_min and r <= scan_msg.range_max]
        right_valid = [r for r in [scan_msg.ranges[i] for i in right_front_indices] 
                      if r >= scan_msg.range_min and r <= scan_msg.range_max]
        
        # Wenn keine gültigen Messungen vorhanden sind, keine Hindernisse
        if not front_valid:
            self.obstacle_detected = False
            return
            
        # Prüfe, ob Hindernisse im Vordergrund sind
        min_front_distance = min(front_valid) if front_valid else float('inf')
        
        if min_front_distance < self.obstacle_distance_threshold:
            self.obstacle_detected = True
            
            # Entscheide, in welche Richtung ausgewichen werden soll
            # Wähle die Richtung mit mehr Platz
            avg_left_distance = sum(left_valid) / len(left_valid) if left_valid else 0
            avg_right_distance = sum(right_valid) / len(right_valid) if right_valid else 0
            
            self.obstacle_direction = 'links' if avg_left_distance > avg_right_distance else 'rechts'
            self.get_logger().info(f'Hindernis erkannt: Distanz={min_front_distance:.2f}m, Ausweichrichtung={self.obstacle_direction}')
        else:
            self.obstacle_detected = False

    def image_callback(self, msg):
        try:
            # Konvertiere ROS-Bild zu OpenCV-Format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Konvertiere Bild zu HSV für bessere Farberkennung
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Erstelle Maske für rote Farbe (zwei Bereiche)
            mask1 = cv2.inRange(hsv_image, self.lower_red1, self.upper_red1)
            mask2 = cv2.inRange(hsv_image, self.lower_red2, self.upper_red2)
            red_mask = cv2.bitwise_or(mask1, mask2)
            
            # Finde Konturen in der Maske
            contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
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
                    
                    # Speichere die letzte bekannte Position des Ziels
                    self.last_target_position = (cx, cy)
                    
                    # Zeichne Kreis um Schwerpunkt zur Visualisierung
                    cv2.circle(cv_image, (cx, cy), 5, (0, 255, 0), -1)
                    
                    # Bildmitte bestimmen
                    height, width, _ = cv_image.shape
                    center_x = width // 2
                    
                    # Berechne horizontalen Fehler
                    error_x = cx - center_x
                    
                    # Schätze die Entfernung basierend auf der Größe des Objekts
                    normalized_area = largest_area / (width * height)
                    estimated_distance = 1.0 / (normalized_area * 10 + 0.1)  # Anpassen nach Bedarf
                    
                    self.get_logger().info(f'Rotes Objekt gefunden: Fläche={largest_area}, ' +
                                           f'Geschätzte Distanz={estimated_distance:.2f}m')
            else:
                # Kein rotes Objekt gefunden
                self.target_detected = False
                
            # Debug-Anzeige (nur aktivieren, wenn ein Display angeschlossen ist)
            try:
                cv2.imshow("Camera View", cv_image)
                cv2.imshow("Red Mask", red_mask)
                cv2.waitKey(1)
            except Exception as e:
                self.get_logger().warn(f'Kann Bild nicht anzeigen: {str(e)}')
            
        except Exception as e:
            self.get_logger().error(f'Fehler bei der Bildverarbeitung: {str(e)}')
    
    def state_machine(self):
        """Hauptzustandsautomat für die Robotersteuerung"""
        # Initialisiere Nachricht für Bewegungsbefehle
        twist_msg = Twist()
        
        # Aktuelle Zeit für zeitbasierte Aktionen
        current_time = time.time()
        
        # Zustandsmaschine
        if self.state == "SEARCHING":
            if self.target_detected:
                self.state = "APPROACHING"
                self.get_logger().info('Ziel erkannt, nähere mich an')
            else:
                # Langsam drehen, um nach Objekten zu suchen
                twist_msg.angular.z = 0.5
        
        elif self.state == "APPROACHING":
            if not self.target_detected:
                self.state = "SEARCHING"
                self.get_logger().info('Ziel verloren, suche erneut')
            elif self.obstacle_detected:
                self.state = "AVOIDING"
                self.avoidance_start_time = current_time
                self.get_logger().info('Hindernis erkannt, beginne Ausweichmanöver')
            else:
                # Wenn wir das Ziel sehen und kein Hindernis im Weg ist
                if self.last_target_position:
                    height, width = 480, 640  # Standardwerte, falls kein aktuelles Bild
                    center_x = width // 2
                    
                    cx, cy = self.last_target_position
                    error_x = cx - center_x
                    
                    # Ausrichten und vorwärts bewegen
                    twist_msg.angular.z = -float(error_x) / 500  # Proportionale Steuerung
                    twist_msg.linear.x = 0.2  # Vorwärts mit moderater Geschwindigkeit
                    
                    # Schätze die Entfernung basierend auf der Größe des Objekts
                    # Hier vereinfacht: Wenn Laserscanner direkt vor uns nichts erkennt,
                    # gehen wir davon aus, dass wir nah genug sind
                    if self.laser_data:
                        center_idx = len(self.laser_data.ranges) // 2
                        center_range = self.laser_data.ranges[center_idx]
                        if center_range <= self.target_distance + self.distance_tolerance:
                            self.state = "STOPPING"
                            self.get_logger().info('Ziel erreicht!')
        
        elif self.state == "AVOIDING":
            # Ausweichmanöver
            if not self.obstacle_detected:
                # Wenn kein Hindernis mehr erkannt wird, zurück zum Verfolgen des Ziels
                if self.target_detected:
                    self.state = "APPROACHING"
                    self.get_logger().info('Hindernis umfahren, nähere mich wieder dem Ziel')
                else:
                    self.state = "SEARCHING"
                    self.get_logger().info('Hindernis umfahren, suche nach Ziel')
            else:
                # Ausweichstrategie basierend auf erkannter Richtung
                if self.obstacle_direction == 'links':
                    # Nach rechts ausweichen
                    twist_msg.linear.x = 0.1
                    twist_msg.angular.z = -0.5
                else:
                    # Nach links ausweichen
                    twist_msg.linear.x = 0.1
                    twist_msg.angular.z = 0.5
                    
                # Wenn wir zu lange ausweichen, zurück zum Suchen
                if current_time - self.avoidance_start_time > 5.0:
                    self.state = "SEARCHING"
                    self.get_logger().info('Ausweichzeit überschritten, beginne neue Suche')
        
        elif self.state == "TURNING":
            # Vollständige Drehung (ca. 6 Sekunden bei 1.0 rad/s)
            if current_time - self.turn_start_time > 6.0:  
                self.state = "SEARCHING"
                twist_msg.angular.z = 0.0
                self.get_logger().info('Drehung abgeschlossen, zurück zum Suchen')
            else:
                twist_msg.angular.z = 1.0  # Weiterdrehen
        
        elif self.state == "STOPPING":
            # Stoppen, wenn wir am Ziel sind
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            
            # Optionaler Übergang zu einem anderen Zustand nach einer Pause
            # Zum Beispiel könnte der Roboter nach einer Pause wieder mit der Suche beginnen
            if not hasattr(self, 'stop_start_time'):
                self.stop_start_time = current_time
            elif current_time - self.stop_start_time > 3.0:
                self.state = "SEARCHING"
                self.get_logger().info('Ziel erreicht, beginne neue Suche')
                delattr(self, 'stop_start_time')
        
        # Veröffentliche Bewegungsbefehl
        self.cmd_vel_publisher.publish(twist_msg)

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

if _name_ == '_main_':
    main()