#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class MockResetServer(Node):
    def __init__(self):
        super().__init__('mock_reset_server')
        # We maken de echte service server aan waar jouw HMI naar zoekt!
        self.srv = self.create_service(Trigger, '/robot/reset_error', self.reset_callback)
        self.get_logger().info("====================================================")
        self.get_logger().info("✅ Mock Reset Service Server is ONLINE!")
        self.get_logger().info("Wacht op HMI om op de RESET-knop te drukken...")
        self.get_logger().info("====================================================")

    def reset_callback(self, request, response):
        self.get_logger().info("➔ HMI heeft op RESET gedrukt! Vraag ontvangen.")
        response.success = True
        response.message = "Fouten succesvol gewist via de Mock Server!"
        self.get_logger().info("← Antwoord teruggestuurd naar de HMI: Geregeld!")
        return response

def main(args=None):  # <-- Gecorrigeerd: args=None toegevoegd
    rclpy.init(args=args)
    print("--- SCRIPT IS GESTART ---")
    node = MockResetServer()
    try:
        rclpy.spin(node)  # <-- Dit zorgt ervoor dat het script blijft draaien!
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    