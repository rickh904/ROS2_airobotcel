#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

# Importeer de CoordRobot service definitie uit jouw interfaces pakket
from interfaces.srv import CoordRobot 


class PositieTransformatieNode(Node):
    def __init__(self):
        super().__init__('positie_transformatie_node')
        
        # --- CONFIGURATIE ---
        # Positie van het ArUco 0-punt ten opzichte van het Robot 0-punt (in mm)
        self.aruco_offset_x = 34.2   
        self.aruco_offset_y = 340.0  
        self.fixed_robot_z = 0.0

        # Zet deze op True als de grijper precies de verkeerde kant opdraait
        self.invert_yaw = True
        # ---------------------

        self.srv = self.create_service(
            CoordRobot,
            'Coord_Robot',
            self.handle_transformation
        )

        self.get_logger().info('CoordRobot Transformatie Service Server is actief...')

    def calculate_constrained_quaternion(self, yaw_degrees):
        """
        Zorgt dat de hoek binnen het bereik van -90 tot +90 graden blijft.
        Daarna wordt de richting omgedraaid, zodat linksom/rechtsom klopt.
        """

        # 1. Breng de hoek naar [-90, +90]
        yaw_constrained = yaw_degrees % 180.0

        if yaw_constrained > 180.0:
            yaw_constrained -= 360

        # 2. Draairichting omdraaien
        # Voorbeeld:
        #  45 graden wordt -45 graden
        # -45 graden wordt  45 graden
        if self.invert_yaw:
            yaw_constrained = -yaw_constrained

        self.get_logger().info(
            f'Gelimiteerde hoek voor robot: {yaw_constrained:.1f}° '
            f'(Input was: {yaw_degrees:.1f}°)'
        )

        # 3. Omrekenen naar radialen
        yaw_rad = math.radians(yaw_constrained)

        # 4. Bereken quaternion-output zoals jouw Robot.py hem gebruikt
        qx = round(math.cos(yaw_rad / 2.0), 3)
        qy = round(math.sin(yaw_rad / 2.0), 3)

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

        # Omrekenen naar meters
        response.robot_x = round(x_robot_mm / 1000.0, 3)
        response.robot_y = round(y_robot_mm / 1000.0, 3)
        response.robot_z = self.fixed_robot_z

        # Bereken rotatie
        qx, qy, qz, qw = self.calculate_constrained_quaternion(yaw_aruco)

        response.qx = qx
        response.qy = qy
        response.qz = qz
        response.qw = qw

        self.get_logger().info(
            f'Omgerekend naar Robot: '
            f'X={response.robot_x:.3f}m, '
            f'Y={response.robot_y:.3f}m | '
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