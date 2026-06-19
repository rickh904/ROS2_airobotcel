import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped


class CoordinateTransformNode(Node):
    def __init__(self):
        super().__init__('coordinate_transform_node')

        # Input van vision:
        # x = pixel_x van midden object
        # y = pixel_y van midden object
        # z = niet gebruikt, mag 0 zijn
        self.pixel_subscriber = self.create_subscription(
            Point,
            '/detected_object_pixel',
            self.pixel_callback,
            10
        )

        # Output naar controller / MoveIt:
        # robot pick-pose in base_link
        self.pick_pose_publisher = self.create_publisher(
            PoseStamped,
            '/pick_pose',
            10
        )

        # Cameraresolutie. Later aanpassen aan echte OAK-camera output.
        self.image_width = 640.0
        self.image_height = 480.0

        # Werkveld in robotcoördinaten.
        # Deze waarden zijn nu voorbeeldwaarden en moeten later gemeten worden.
        self.robot_x_min = 0.20
        self.robot_x_max = 0.45
        self.robot_y_min = -0.20
        self.robot_y_max = 0.20

        # Vaste grijphoogte boven het vlakke veld.
        self.pick_z = 0.04

        self.get_logger().info('Coordinate transform node gestart')
        self.get_logger().info('Input : /detected_object_pixel geometry_msgs/Point')
        self.get_logger().info('Output: /pick_pose geometry_msgs/PoseStamped')

    def pixel_callback(self, msg):
        pixel_x = msg.x
        pixel_y = msg.y

        if not self.pixel_is_valid(pixel_x, pixel_y):
            self.get_logger().warn(
                f'Ongeldige pixelpositie ontvangen: ({pixel_x}, {pixel_y})'
            )
            return

        robot_x, robot_y, robot_z = self.pixel_to_robot(pixel_x, pixel_y)

        pick_pose = PoseStamped()
        pick_pose.header.stamp = self.get_clock().now().to_msg()
        pick_pose.header.frame_id = 'base_link'

        pick_pose.pose.position.x = robot_x
        pick_pose.pose.position.y = robot_y
        pick_pose.pose.position.z = robot_z

        # Vaste oriëntatie van de gripper.
        # Later kan MoveIt/controller dit aanpassen.
        pick_pose.pose.orientation.x = 0.0
        pick_pose.pose.orientation.y = 0.0
        pick_pose.pose.orientation.z = 0.0
        pick_pose.pose.orientation.w = 1.0

        self.pick_pose_publisher.publish(pick_pose)

        self.get_logger().info(
            f'Pixel ({pixel_x:.1f}, {pixel_y:.1f}) -> '
            f'Robot ({robot_x:.3f}, {robot_y:.3f}, {robot_z:.3f})'
        )

    def pixel_is_valid(self, pixel_x, pixel_y):
        if pixel_x < 0 or pixel_x > self.image_width:
            return False

        if pixel_y < 0 or pixel_y > self.image_height:
            return False

        return True

    def pixel_to_robot(self, pixel_x, pixel_y):
        robot_x = self.robot_x_min + (pixel_x / self.image_width) * (
            self.robot_x_max - self.robot_x_min
        )

        robot_y = self.robot_y_min + (pixel_y / self.image_height) * (
            self.robot_y_max - self.robot_y_min
        )

        return robot_x, robot_y, self.pick_z


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
