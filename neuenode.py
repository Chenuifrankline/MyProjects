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
        self.initial_scan_completed = False
        self.initial_scan_start_time = None
        self.is_turning = False
        self.turn_start_time = None
        self.state = "INITIAL_SCAN"  # INITIAL_SCAN, SEARCHING, APPROACHING, STOPPING, TURNING
        
        # Timer für Start-Delay (lässt dem System Zeit zum Initialisieren)
        self.create_timer(2.0, self.start_initial_scan)
        
        # Parameter für die Farbdetektion (Rot in HSV)
        # Engerer Bereich für Rot als zuvor, um Fehlerkennungen zu reduzieren
        self.lower_red1 = np.array([0, 120, 120])
        self.upper_red1 = np.array([8, 255, 255])
        self.lower_red2 = np.array([165, 120, 120])
        self.upper_red2 = np.array([180, 255, 255])
        
        # Schwellenwerte
        self.min_contour_area = 800  # Erhöht, um kleine Objekte zu ignorieren
        self.target_distance = 0.2  # Zieldistanz in Metern (20 cm)
        self.distance_tolerance = 0.05  # Toleranz in Metern
        
        # Genauigkeitsparameter für Viereckerkennung
        self.rectangle_epsilon = 0.05  # Relative Genauigkeit für Konturapproximation
        self.min_rectangle_similarity = 0.75  # Mindestähnlichkeit für Rechteck
        
        self.get_logger().info('Color Detector Node wurde initialisiert')
    
    def start_initial_scan(self):
        """Startet den ersten 360°-Scan nach dem roten Viereck"""
        self.get_logger().info('Starte initialen 360°-Scan nach rotem Viereck...')
        self.state = "INITIAL_SCAN"
        self.initial_scan_start_time = time.time()
        self.initial_scan_completed = False
        
        # Timer deaktivieren, damit die Funktion nicht erneut aufgerufen wird
        self.create_timer(0.1, lambda: None, cancel=True)

    def is_rectangle(self, contour):
        """Überprüft, ob die Kontur ein Viereck ist"""
        # Perimeter der Kontur berechnen
        perimeter = cv2.arcLength(contour, True)
        
        # Kontur mit einem bestimmten Epsilon annähern
        epsilon = self.rectangle_epsilon * perimeter
        approx = cv2.approxPolyDP(contour, epsilon, True)
        
        # Ein Viereck hat 4 Ecken
        if len(approx) == 4:
            # Berechne die Rechteckigkeit der Kontur (wie ähnlich ist sie einem echten Rechteck)
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            rect_area = cv2.contourArea(box)
            contour_area = cv2.contourArea(contour)
            
            # Wenn die Fläche der Kontur mindestens 75% der Rechteckfläche beträgt,
            # wird es als Rechteck betrachtet
            if contour_area > 0 and rect_area > 0:
                similarity = contour_area / rect_area
                return similarity > self.min_rectangle_similarity
        
        return False

    def image_callback(self, msg):
        try:
            # Konvertiere ROS-Bild zu OpenCV-Format
            # Explizites Konvertieren zu bgr8, falls die Kamera ein anderes Format sendet
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Debug-Info ausgeben (nur gelegentlich, um Log nicht zu überfüllen)
            if np.random.random() < 0.01:  # Nur ca. 1% der Frames loggen
                self.get_logger().info(f'Bild empfangen: Format={msg.encoding}, Größe={msg.width}x{msg.height}')
            
            # Konvertiere Bild zu HSV für bessere Farberkennung
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # Erstelle Maske für rote Farbe (zwei Bereiche)
            mask1 = cv2.inRange(hsv_image, self.lower_red1, self.upper_red1)
            mask2 = cv2.inRange(hsv_image, self.lower_red2, self.upper_red2)
            red_mask = cv2.bitwise_or(mask1, mask2)
            
            # Morphologische Operationen zur Rauschunterdrückung
            kernel = np.ones((5, 5), np.uint8)
            red_mask = cv2.erode(red_mask, kernel, iterations=1)
            red_mask = cv2.dilate(red_mask, kernel, iterations=2)
            
            # Finde Konturen in der Maske
            contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            # Initialisiere Nachricht für Bewegungsbefehle
            twist_msg = Twist()
            
            # INITIAL_SCAN: Vollständige Drehung zu Beginn
            if self.state == "INITIAL_SCAN":
                if not self.initial_scan_completed:
                    # Vollständige Drehung (ca. 8 Sekunden bei 0.75 rad/s)
                    current_time = time.time()
                    if current_time - self.initial_scan_start_time > 8.0:
                        self.get_logger().info('Initialer Scan abgeschlossen')
                        self.initial_scan_completed = True
                        self.state = "SEARCHING"
                        twist_msg.angular.z = 0.0
                    else:
                        twist_msg.angular.z = 0.75  # Kontinuierlich drehen
                else:
                    self.state = "SEARCHING"
            
            # TURNING: Dreht sich einmal im Kreis vor dem roten Viereck
            elif self.state == "TURNING":
                current_time = time.time()
                # Vollständige Drehung (ca. 6 Sekunden bei 1.0 rad/s)
                if current_time - self.turn_start_time > 6.0:  
                    self.state = "SEARCHING"
                    twist_msg.angular.z = 0.0
                    self.get_logger().info('Drehung abgeschlossen, zurück zum Suchen')
                else:
                    twist_msg.angular.z = 1.0  # Weiterdrehen
            
            # SEARCHING oder APPROACHING: Suche/Fahre zu rotem Viereck
            else:
                # Initialisiere Variablen für Zielverfolgung
                target_contour = None
                largest_rect_area = 0
                
                # Alle Konturen durchgehen und nach roten Vierecken suchen
                for contour in contours:
                    area = cv2.contourArea(contour)
                    # Ignoriere kleine Konturen
                    if area > self.min_contour_area:
                        # Wenn Kontur ein Viereck ist
                        if self.is_rectangle(contour):
                            if area > largest_rect_area:
                                largest_rect_area = area
                                target_contour = contour
                                
                                # Debug: Zeichne erkannte Vierecke
                                # Umrahme das Viereck im Original-Bild
                                rect = cv2.minAreaRect(contour)
                                box = cv2.boxPoints(rect)
                                box = np.int0(box)
                                cv2.drawContours(cv_image, [box], 0, (0, 255, 0), 2)
                
                # Wenn ein rotes Viereck gefunden wurde
                if target_contour is not None:
                    # Ziel gefunden
                    self.target_detected = True
                    
                    # Berechne Momentane (Schwerpunkt)
                    M = cv2.moments(target_contour)
                    if M["m00"] > 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        
                        # Zeichne Kreuz an Schwerpunkt
                        cv2.drawMarker(cv_image, (cx, cy), (0, 0, 255), 
                                    markerType=cv2.MARKER_CROSS, markerSize=20, thickness=2)
                        
                        # Bildmitte bestimmen
                        height, width, _ = cv_image.shape
                        center_x = width // 2
                        
                        # Berechne horizontalen Fehler
                        error_x = cx - center_x
                        
                        # Schätze die Entfernung basierend auf der Größe des Objekts
                        normalized_area = largest_rect_area / (width * height)
                        # Verbesserte Distanzformel, kalibriert für 20cm Zieldistanz
                        # Diese Formel kann angepasst werden, basierend auf realen Tests
                        estimated_distance = 0.8 / (normalized_area * 20 + 0.1)
                        
                        # Logge erkanntes Objekt und Distanz (nicht zu häufig)
                        if np.random.random() < 0.05:  # Weniger häufig loggen
                            self.get_logger().info(f'Rotes Viereck gefunden: Fläche={largest_rect_area}, ' +
                                                f'Geschätzte Distanz={estimated_distance:.2f}m')
                        
                        if self.state == "SEARCHING" or self.state == "APPROACHING":
                            # Wenn wir das Ziel noch nicht erreicht haben
                            if estimated_distance > self.target_distance + self.distance_tolerance:
                                self.state = "APPROACHING"
                                # Ausrichten und vorwärts bewegen
                                twist_msg.angular.z = -float(error_x) / 500  # Proportionale Steuerung
                                
                                # Geschwindigkeit verringern, wenn wir näher kommen
                                if estimated_distance < 0.5:
                                    twist_msg.linear.x = 0.1  # Langsamer
                                else:
                                    twist_msg.linear.x = 0.2  # Normale Geschwindigkeit
                            else:
                                # Wir sind am Ziel
                                self.state = "STOPPING"
                                twist_msg.linear.x = 0.0
                                twist_msg.angular.z = 0.0
                                self.get_logger().info('Ziel erreicht! (20cm Abstand)')
                                
                                # Nach dem Anhalten beginnen wir zu drehen
                                self.state = "TURNING"
                                self.turn_start_time = time.time()
                                twist_msg.angular.z = 1.0  # Start der Drehung
                else:
                    # Kein rotes Viereck gefunden
                    self.target_detected = False
                    if self.state == "SEARCHING":
                        # Langsam drehen, um nach Objekten zu suchen
                        twist_msg.angular.z = 0.5
                        if np.random.random() < 0.05:  # Weniger häufig loggen
                            self.get_logger().info('Kein rotes Viereck gefunden, weiter suchen...')
            
            # Debug-Anzeige (nur aktivieren, wenn ein Display angeschlossen ist)
            # Für Headless-Betrieb können Sie diese Zeilen auskommentieren
            try:
                # Zeichne den aktuellen Zustand ins Bild
                cv2.putText(cv_image, f"State: {self.state}", (10, 30), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
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


    def image_callback(self, msg):
        try:
            # Konvertiere ROS-Bild zu OpenCV-Format
            # Explizites Konvertieren zu bgr8, falls die Kamera ein anderes Format sendet
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Debug-Info ausgeben (nur gelegentlich, um Log nicht zu überfüllen)
            if np.random.random() < 0.01:  # Nur ca. 1% der Frames loggen
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
                                
                                # Nach dem Anhalten beginnen wir zu drehen
                                self.state = "TURNING"
                                self.turn_start_time = time.time()
                                twist_msg.angular.z = 1.0  # Start der Drehung
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