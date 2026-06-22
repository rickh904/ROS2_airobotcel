import rclpy
from rclpy.node import Node

from airobot_interfaces.srv import CameraToRobot


class CoordinateTransformNode(Node):
    def __init__(self):
        super().__init__('coordinate_transform_node')

        self.service = self.create_service(
            CameraToRobot,
            '/camera_to_robot',
            self.camera_to_robot_callback
        )

        # Simpele camera-naar-robot kalibratie.
        # Deze waarden moeten later echt gemeten worden.
        self.offset_x = 0.30
        self.offset_y = 0.00
        self.offset_z = 0.00

        # Richting van camera-assen ten opzichte van robot-assen.
        # Zet naar -1.0 als een as omgekeerd blijkt te zijn.
        self.camera_x_direction = 1.0
        self.camera_y_direction = 1.0

        # Omdat het werkveld vlak is, gebruiken we vaste pickhoogte.
        self.pick_z = 0.04

        # Veilig werkgebied in robotcoördinaten.
        self.safe_x_min = 0.15
        self.safe_x_max = 0.55
        self.safe_y_min = -0.30
        self.safe_y_max = 0.30
        self.safe_z_min = 0.00
        self.safe_z_max = 0.20

        self.get_logger().info('Coordinate transform service gestart')
        self.get_logger().info('Service: /camera_to_robot')
        self.get_logger().info('Input : camera_point in camera-coordinaten')
        self.get_logger().info('Output: robot_pose in base_link')

    def camera_to_robot_callback(self, request, response):
        camera_x = request.camera_point.x
        camera_y = request.camera_point.y
        camera_z = request.camera_point.z

        robot_x, robot_y, robot_z = self.camera_to_robot(
            camera_x,
            camera_y,
            camera_z
        )

        if not self.robot_pose_is_safe(robot_x, robot_y, robot_z):
            response.success = False
            response.message = (
                f'Robotpositie buiten veilig werkgebied: '
                f'x={robot_x:.3f}, y={robot_y:.3f}, z={robot_z:.3f}'
            )

            self.get_logger().warn(response.message)
            return response

        response.robot_pose.header.stamp = self.get_clock().now().to_msg()
        response.robot_pose.header.frame_id = 'base_link'

        response.robot_pose.pose.position.x = robot_x
        response.robot_pose.pose.position.y = robot_y
        response.robot_pose.pose.position.z = robot_z

        # Vaste gripper-oriëntatie.
        # Later kan MoveIt/controller dit aanpassen.
        response.robot_pose.pose.orientation.x = 0.0
        response.robot_pose.pose.orientation.y = 0.0
        response.robot_pose.pose.orientation.z = 0.0
        response.robot_pose.pose.orientation.w = 1.0

        response.success = True
        response.message = 'Camera-coordinaten omgerekend naar robotcoordinaten'

        self.get_logger().info(
            f'Camera ({camera_x:.3f}, {camera_y:.3f}, {camera_z:.3f}) -> '
            f'Robot ({robot_x:.3f}, {robot_y:.3f}, {robot_z:.3f})'
        )

        return response

    def camera_to_robot(self, camera_x, camera_y, camera_z):
        robot_x = self.offset_x + self.camera_x_direction * camera_x
        robot_y = self.offset_y + self.camera_y_direction * camera_y

        # Vlak werkveld: z is vaste pickhoogte.
        robot_z = self.pick_z

        return robot_x, robot_y, robot_z

    def robot_pose_is_safe(self, robot_x, robot_y, robot_z):
        if robot_x < self.safe_x_min or robot_x > self.safe_x_max:
            return False

        if robot_y < self.safe_y_min or robot_y > self.safe_y_max:
            return False

        if robot_z < self.safe_z_min or robot_z > self.safe_z_max:
            return False

        return True


def main(args=None):
    rclpy.init(args=args)

    node = CoordinateTransformNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
