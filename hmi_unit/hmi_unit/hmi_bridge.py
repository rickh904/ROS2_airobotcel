# hmi_bridge.py
import rclpy
from rclpy.node import Node
# We gebruiken nu ALLEEN standaard, ingebouwde ROS2 berichten:
from std_msgs.msg import String, Int32MultiArray, Float32, Bool
from sensor_msgs.msg import Image
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

import cv2
from cv_bridge import CvBridge
from PyQt5.QtGui import QImage
from PyQt5.QtCore import QObject, pyqtSignal

class HmiBridge(Node, QObject):
    ros_image_received = pyqtSignal(QImage)
    ros_state_received = pyqtSignal(str)
    ros_counts_received = pyqtSignal(list)
    ros_error_received = pyqtSignal(str)

    def __init__(self):
        Node.__init__(self, 'hmi_node')
        QObject.__init__(self)
        
        self.bridge = CvBridge()

        # --- ROS2 Subscribers ---
        self.img_sub = self.create_subscription(Image, 'cam', self.image_callback, 10)
        self.status_sub = self.create_subscription(String, '/system/state', self.status_callback, 10)
        self.count_sub = self.create_subscription(Int32MultiArray, '/system/sorter_counts', self.count_callback, 10)
        self.error_sub = self.create_subscription(String, '/system/errors', self.error_callback, 10)

        # --- ROS2 Publishers (We sturen nu alles tijdelijk via topics!) ---
        self.cmd_pub = self.create_publisher(String, '/hmi/commands', 10)
        self.speed_pub = self.create_publisher(Float32, '/robot/speed_limit', 10)
        
        # Dit is de ingebouwde ROS2 service voor parameters (werkt altijd!)
        self.param_client = self.create_client(SetParameters, '/vision/set_parameters')

    # --- Callbacks voor binnengekomende ROS2 data ---
    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, channel = cv_img.shape
            bytes_per_line = channel * width
            qt_img = QImage(cv_img.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            self.ros_image_received.emit(qt_img)
        except Exception as e:
            self.get_logger().error(f"Fout bij camera-beeldstroom: {e}")

    def status_callback(self, msg):
        self.ros_state_received.emit(msg.data)

    def count_callback(self, msg):
        self.ros_counts_received.emit(list(msg.data))

    def error_callback(self, msg):
        self.ros_error_received.emit(msg.data)

    # --- TIJDELIJKE TESTLOGICA VOOR DE KNOPPEN ---
    # In plaats van te crashen op Actions/Services, publiceren we nu simpele Strings of loggen we de actie.

    def trigger_start_sort(self):
        """Tijdelijk: Publiceert 'START' op het commandotopic om te testen."""
        self.get_logger().info("[TEST] START knop ingedrukt!")
        msg = String()
        msg.data = "START_SORT"
        self.cmd_pub.publish(msg)

    def trigger_stop_sort(self):
        """Tijdelijk: Publiceert 'STOP' op het commandotopic om te testen."""
        self.get_logger().info("[TEST] STOP knop ingedrukt!")
        msg = String()
        msg.data = "STOP"
        self.cmd_pub.publish(msg)

    def trigger_home_action(self):
        """Tijdelijk: Publiceert 'HOME' op het commandotopic om te testen."""
        self.get_logger().info("[TEST] HOME knop ingedrukt!")
        msg = String()
        msg.data = "GO_HOME"
        self.cmd_pub.publish(msg)

    def trigger_reset_service(self):
        """Tijdelijk: Publiceert 'RESET' op het commandotopic om te testen."""
        self.get_logger().info("[TEST] RESET knop ingedrukt!")
        msg = String()
        msg.data = "RESET_ERROR"
        self.cmd_pub.publish(msg)

    # --- Parameters & Snelheid (Standaard ROS2, dit werkt sowieso) ---
    def publish_speed(self, speed_value):
        msg = Float32()
        msg.data = speed_value
        self.speed_pub.publish(msg)

    def update_ai_parameter(self, value):
        if self.param_client.wait_for_service(timeout_sec=0.5):
            req = SetParameters.Request()
            param_val = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
            req.parameters = [Parameter(name='confidence_threshold', value=param_val)]
            self.param_client.call_async(req)