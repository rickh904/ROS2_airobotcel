#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

# Importeer de CoordRobot service definitie uit jouw interfaces pakket
from interfaces.srv import CoordRobot 

class PositieTransformatieNode(Node):
    def __init__(self):
        super().__init__('positie_transformatie_node')
        
        # --- CONFIGURATIE (Makkelijk aan te passen) ---
        # Positie van het ArUco 0-punt ten opzichte van het Robot 0-punt (in mm)
        self.aruco_offset_x = 41.2   
        self.aruco_offset_y = 182.3  
        self.fixed_robot_z = 0.0     # Vaste z-hoogte in meters voor het oppakken
        # -----------------------------------------------

        # Service Server aanmaken
        self.srv = self.create_service(CoordRobot, 'Coord_Robot', self.handle_transformation)
        self.get_logger().info('CoordRobot Transformatie Service Server is actief...')

    def calculate_constrained_quaternion(self, yaw_degrees):
        """
        Zorgt dat de hoek binnen het bereik van -90 tot +90 graden blijft,
        en zet dit om naar de specifieke x en y waarden (met z=0 en w=0).
        """
        # 1. Breng de hoek naar het bereik [0, 180) en verschuif naar [-90, +90]
        yaw_constrained = yaw_degrees % 180.0
        if yaw_constrained > 90.0:
            yaw_constrained -= 180.0
            
        self.get_logger().info(f'Gelimiteerde hoek voor robot: {yaw_constrained:.1f}° (Input was: {yaw_degrees:.1f}°)')

        # Omrekenen naar radialen
        yaw_rad = math.radians(yaw_constrained)

        # 2. Bereken de x en y componenten en rond ze direct af op 3 decimalen
        qx = round(math.cos(yaw_rad / 2.0), 3)
        qy = round(math.sin(yaw_rad / 2.0), 3)
        
        # 3. Dwing z en w af op 0.0 voor jouw specifieke action server
        qz = 0.0
        qw = 0.0

        return qx, qy, qz, qw

    def handle_transformation(self, request, response):
        x_aruco = request.x
        y_aruco = request.y
        yaw_aruco = request.yaw

        # Assen mapping van ArUco naar Robot (in mm)
        x_robot_mm = self.aruco_offset_x + (-y_aruco)
        y_robot_mm = self.aruco_offset_y + (-x_aruco)

        # Omrekenen naar meters voor de Response (ook afgerond op 3 decimalen)
        response.robot_x = round(x_robot_mm / 1000.0, 3)
        response.robot_y = round(y_robot_mm / 1000.0, 3)
        response.robot_z = self.fixed_robot_z

        # Bereken de begrensde rotatie en zet om naar de gewenste rotation output
        qx, qy, qz, qw = self.calculate_constrained_quaternion(yaw_aruco)

        response.qx = qx
        response.qy = qy
        response.qz = qz
        response.qw = qw

        self.get_logger().info(
            f'Omgerekend naar Robot: X={response.robot_x:.3f}m, Y={response.robot_y:.3f}m | '
            f'Rotation: x: {qx:.3f}, y: {qy:.3f}, z: {qz:.3f}, w: {qw:.3f}'
        )
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = PositieTransformatieNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()