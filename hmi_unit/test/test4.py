#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

# Importeer de benodigde ROS 2 parameter interfaces
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import SetParametersResult

class MockVisionNode(Node):
    def __init__(self):
        super().__init__('mock_vision_node')
        
        # --- PARAMETER SERVICE SERVER ---
        # Deze service luistert naar wijzigingen die vanaf de HMI worden gestuurd
        self._param_server = self.create_service(
            SetParameters,
            '/vision/set_parameters',
            self.execute_param_callback
        )
        
        self.get_logger().info("====================================================")
        self.get_logger().info("🎯 Mock Vision Parameter Server is ONLINE!")
        self.get_logger().info("Luistert naar instellingen vanaf de HMI...")
        self.get_logger().info("  ➡️  Service endpoint: /vision/set_parameters")
        self.get_logger().info("====================================================")

    def execute_param_callback(self, request, response):
        """Wordt getriggerd zodra je de spinbox op de HMI aanpast."""
        for param in request.parameters:
            if param.name == 'confidence_threshold':
                nieuwe_waarde = param.value.double_value
                self.get_logger().info(f"🎯 [AI VISION] Parameter ontvangen! 'confidence_threshold' ingesteld op: {nieuwe_waarde}")

        # Geef een netjes antwoord terug aan de HMI Bridge zodat de GUI weet dat het gelukt is
        res = SetParametersResult(successful=True, reason="Threshold succesvol aangepast")
        response.results = [res]
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockVisionNode()
    
    try:
        # Spin houdt de node actief zodat hij op service calls kan blijven reageren
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
        
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
