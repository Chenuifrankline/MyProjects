#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

def callback(data):
    # Extrahiere die x-Position aus den Odom-Daten
    x_position = data.pose.pose.position.x
    rospy.loginfo("Aktuelle x-Position: %f", x_position)

    # Bewegungsbefehl
    move_cmd = Twist()

    # Logik: Vorwärts bewegen, wenn x < 2.0
    if x_position < 2.0:
        move_cmd.linear.x = 0.2  # Geschwindigkeit
        rospy.loginfo("Bewege vorwärts")
    else:
        move_cmd.linear.x = 0.0  # Anhalten
        rospy.loginfo("Anhalten")

    # Befehl veröffentlichen
    pub.publish(move_cmd)

if __name__ == '__main__':
    try:
        # ROS-Knoten initialisieren
        rospy.init_node('simple_turtlebot_controller', anonymous=True)

        # Publisher erstellen
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber erstellen
        rospy.Subscriber('/odom', Odometry, callback)

        rospy.loginfo("TurtleBot Controller gestartet")
        
        # ROS-Node laufen lassen
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
