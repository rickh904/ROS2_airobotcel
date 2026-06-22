import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped


class CoordinateTransformNode(Node):
    def __init__(self):
        super().__init__('coordinate_transform_node')

        # Input 1:
        # Vision geeft pixelcoördinaten van het midden van het object.
        self.pixel_subscriber = self.create_subscription(
            Point,
            '/detected_object_pixel',
            self.pixel_callback,
            10
        )

        # Input 2:
        # Vision/OAK geeft al 3D-camera-coördinaten.
        self.camera_subscriber = self.create_subscription(
            Point,
            '/detected_object_camera',
            self.camera_callback,
            10
        )

        # Output:
        # Robot pick-pose in robotbasis-frame.
        self.pick_pose_publisher = self.create_publisher(
            PoseStamped,
            '/pick_pose',
            10
        )

        # -----------------------------
        # CAMERA / BEELD INSTELLINGEN
        # -----------------------------

        # Resolutie van het camerabeeld.
        # Later aanpassen aan echte OAK output.
        self.image_width = 640.0
        self.image_height = 480.0

        # -----------------------------
        # PIXEL NAAR ROBOT KALIBRATIE
        # -----------------------------

        # Robotcoördinaten van het vlakke werkveld.
        # Dit zijn tijdelijke voorbeeldwaarden.
        # Deze moeten later fysiek gemeten worden.
        self.robot_x_min = 0.20
        self.robot_x_max = 0.45
        self.robot_y_min = -0.20
        self.robot_y_max = 0.20

        # -----------------------------
        # CAMERA NAAR ROBOT KALIBRATIE
        # -----------------------------

        # Simpele offset tussen camera-frame en robot-frame.
        # Deze waarden moeten later gemeten worden.
        self.camera_to_robot_offset_x = 0.30
        self.camera_to_robot_offset_y = 0.00
        self.camera_to_robot_offset_z = 0.00

        # Als de camera-assen anders staan dan de robot-assen,
        # kun je hier later tekens omdraaien.
        self.camera_x_direction = 1.0
        self.camera_y_direction = 1.0

        # Vaste grijphoogte boven het vlakke veld.
        self.pick_z = 0.04

        # Veiligheidsgrenzen werkgebied robot.
        # Als een berekende pose buiten dit gebied valt, wordt hij niet gepubliceerd.
        self.safe_x_min = 0.15
        self.safe_x_max = 0.55
        self.safe_y_min = -0.30
        self.safe_y_max = 0.30
        self.safe_z_min = 0.00
        self.safe_z_max = 0.20

        self.get_logger().info('Coordinate transform node gestart')
        self.get_logger().info('Input 1: /detected_object_pixel geometry_msgs/Point')
        self.get_logger().info('Input 2: /detected_object_camera geometry_msgs/Point')
        self.get_logger().info('Output : /pick_pose geometry_msgs/PoseStamped in base_link')

    def pixel_callback(self, msg):
        """
        Wordt gebruikt als vision pixelcoördinaten geeft.
        msg.x = pixel_x
        msg.y = pixel_y
        msg.z = niet gebruikt
        """

        pixel_x = msg.x
        pixel_y = msg.y

        if not self.pixel_is_valid(pixel_x, pixel_y):
            self.get_logger().warn(
                f'Ongeldige pixelpositie ontvangen: ({pixel_x}, {pixel_y})'
            )
            return

        robot_x, robot_y, robot_z = self.pixel_to_robot(pixel_x, pixel_y)

        self.publish_pick_pose(
            robot_x,
            robot_y,
            robot_z,
            source='pixel'
        )

        self.get_logger().info(
            f'Pixel ({pixel_x:.1f}, {pixel_y:.1f}) -> '
            f'Robot ({robot_x:.3f}, {robot_y:.3f}, {robot_z:.3f})'
        )

    def camera_callback(self, msg):
        """
        Wordt gebruikt als vision/OAK al 3D-camera-coördinaten geeft.
        msg.x = camera_x
        msg.y = camera_y
        msg.z = camera_z
        """

        camera_x = msg.x
        camera_y = msg.y
        camera_z = msg.z

        robot_x, robot_y, robot_z = self.camera_to_robot(
            camera_x,
            camera_y,
            camera_z
        )

        self.publish_pick_pose(
            robot_x,
            robot_y,
            robot_z,
            source='camera'
        )

        self.get_logger().info(
            f'Camera ({camera_x:.3f}, {camera_y:.3f}, {camera_z:.3f}) -> '
            f'Robot ({robot_x:.3f}, {robot_y:.3f}, {robot_z:.3f})'
        )

    def pixel_to_robot(self, pixel_x, pixel_y):
        """
        Zet pixelcoördinaten om naar robotcoördinaten.
        Dit is een 2D lineaire mapping voor een vlak werkveld.
        """

        robot_x = self.robot_x_min + (pixel_x / self.image_width) * (
            self.robot_x_max - self.robot_x_min
        )

        robot_y = self.robot_y_min + (pixel_y / self.image_height) * (
            self.robot_y_max - self.robot_y_min
        )

        robot_z = self.pick_z

        return robot_x, robot_y, robot_z

    def camera_to_robot(self, camera_x, camera_y, camera_z):
        """
        Zet 3D-camera-coördinaten om naar robotcoördinaten.
        Dit is een simpele offset-transform.
        Later kan dit vervangen worden door een echte TF-transform.
        """

        robot_x = self.camera_to_robot_offset_x + (
            self.camera_x_direction * camera_x
        )

        robot_y = self.camera_to_robot_offset_y + (
            self.camera_y_direction * camera_y
        )

        # Omdat het werkveld vlak is, gebruiken we voorlopig vaste pickhoogte.
        robot_z = self.pick_z

        return robot_x, robot_y, robot_z

    def publish_pick_pose(self, robot_x, robot_y, robot_z, source):
        """
        Publiceert de robot pick-pose als PoseStamped.
        """

        if not self.robot_pose_is_safe(robot_x, robot_y, robot_z):
            self.get_logger().warn(
                f'Berekende robotpositie buiten veilig werkgebied: '
                f'({robot_x:.3f}, {robot_y:.3f}, {robot_z:.3f})'
            )
            return

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
            f'Pick-pose gepubliceerd vanuit {source}-input op /pick_pose'
        )

    def pixel_is_valid(self, pixel_x, pixel_y):
        """
        Controleert of pixel binnen het camerabeeld valt.
        """

        if pixel_x < 0.0 or pixel_x > self.image_width:
            return False

        if pixel_y < 0.0 or pixel_y > self.image_height:
            return False

        return True

    def robot_pose_is_safe(self, robot_x, robot_y, robot_z):
        """
        Controleert of de berekende robotpositie binnen het veilige werkgebied valt.
        """

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
