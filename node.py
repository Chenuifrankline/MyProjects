class ColorDetectorNode(Node):
    def __init__(self):
        super().__init__('color_detector_node')
        self.get_logger().info("Node initializing...")
        
        from rclpy.qos import qos_profile_sensor_data
        
        # Subscriber mit QoS-Profil
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile=qos_profile_sensor_data
        )
        self.get_logger().info(f"Created subscriber to: {self.image_subscription.topic_name}")
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info("Node ready!")

    def image_callback(self, msg):
        self.get_logger().info("Image received!")  # Grundlegende Bestätigung
        # Rest des Codes...