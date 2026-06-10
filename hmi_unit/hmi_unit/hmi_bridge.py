# hmi_bridge.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

import cv2
from cv_bridge import CvBridge
from PyQt5.QtCore import QObject, pyqtSignal, QImage

class HmiBridge(Node, QObject):
    # Signals om data VEILIG naar de hoofd-GUI thread te pushen
    ros_image_received = pyqtSignal(QImage)
    ros_state_received = pyqtSignal(str)
    ros_counts_received = pyqtSignal(list)

    def __init__(self):
        # Initialiseer zowel de ROS2 Node als de QObject
        Node.__init__(self, 'hmi_node')
        QObject.__init__(self)
        
        self.bridge = CvBridge()

        # --- ROS2 Subscribers ---
        self.img_sub = self.create_subscription(Image, '/vision/ai_output_image', self.image_callback, 10)
        self.status_sub = self.create_subscription(String, '/system/state', self.status_callback, 10)
        self.count_sub = self.create_subscription(Int32MultiArray, '/system/sorter_counts', self.count_callback, 10)

        # --- ROS2 Publishers & Service Clients ---
        self.cmd_pub = self.create_publisher(String, '/hmi/commands', 10)
        self.home_client = self.create_client(Trigger, '/robot/go_home')
        self.param_client = self.create_client(SetParameters, '/vision/set_parameters')

    # --- Callbacks: ROS2 Data binnenkrijgen -> PyQt Signal afvuren ---
    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, channel = cv_img.shape
            bytes_per_line = channel * width
            qt_img = QImage(cv_img.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            self.ros_image_received.emit(qt_img)
        except Exception as e:
            self.get_logger().error(f"Fout bij conversie ROS2-beeld: {e}")

    def status_callback(self, msg):
        self.ros_state_received.emit(msg.data)

    def count_callback(self, msg):
        self.ros_counts_received.emit(list(msg.data))

    # --- Actions: GUI interactie opvangen -> ROS2 in sturen ---
    def publish_command(self, cmd_string):
        msg = String()
        msg.data = cmd_string
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"HMI stuurde commando: {cmd_string}")

    def trigger_home_service(self):
        if self.home_client.wait_for_service(timeout_sec=0.5):
            req = Trigger.Requests()
            self.home_client.call_async(req)
            self.get_logger().info("HMI activeerde Manual Override Home Service.")
        else:
            self.get_logger().warn("Robot Home Service niet beschikbaar!")

    def update_ai_parameter(self, value):
        if self.param_client.wait_for_service(timeout_sec=0.5):
            req = SetParameters.Request()
            param_val = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
            req.parameters = [Parameter(name='confidence_threshold', value=param_val)]
            self.param_client.call_async(req)