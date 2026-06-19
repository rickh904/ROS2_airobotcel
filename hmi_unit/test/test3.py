#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import math

class MockCameraPublisher(Node):
    def __init__(self):
        super().__init__('mock_camera_publisher')
        
        # We publiceren op de exact dezelfde topicnaam als je HMI subscriber ('cam')
        self.publisher_ = self.create_publisher(Image, 'cam', 10)
        
        # Timer voor 10 frames per seconde (0.1s interval)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.bridge = CvBridge()
        self.frame_id = 0

        self.get_logger().info("====================================================")
        self.get_logger().info("🎥 Mock Camera Publisher is ONLINE!")
        self.get_logger().info("Streaming een virtuele video op topic: /cam")
        self.get_logger().info("====================================================")

    def timer_callback(self):
        # 1. Maak een zwart achtergrondframe aan (640x480 pixels, 3 kleurkanalen)
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        
        # 2. Bereken een bewegende X-positie voor een cirkel (bouncing effect)
        x_pos = int(320 + 200 * math.sin(self.frame_id * 0.1))
        y_pos = 240
        
        # 3. Teken de cirkel en zet een live teller in beeld
        cv2.circle(frame, (x_pos, y_pos), 50, (0, 255, 0), -1) # Groene cirkel
        cv2.putText(frame, f"OAK-D MOCK CAM - Frame: {self.frame_id}", (30, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
        
        # 4. Converteer het OpenCV frame (BGR8) naar een ROS2 Image Message
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "camera_frame"
            
            # 5. Versturen maar!
            self.publisher_.publish(img_msg)
            self.frame_id += 1
        except Exception as e:
            self.get_logger().error(f"Fout bij converteren van frame: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MockCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()