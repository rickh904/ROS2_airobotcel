import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray, Float32
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger  
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

import cv2
from cv_bridge import CvBridge
from PyQt5.QtGui import QImage
from PyQt5.QtCore import QObject, pyqtSignal

try:
    from interfaces.action import AutoSort, GoHome
    from rclpy.action import ActionClient
    HAS_CUSTOM_INTERFACES = True
except ModuleNotFoundError:
    HAS_CUSTOM_INTERFACES = False

class HmiBridge(Node, QObject):
    ros_image_received = pyqtSignal(QImage)
    ros_state_received = pyqtSignal(str)
    ros_counts_received = pyqtSignal(list)
    ros_error_received = pyqtSignal(str)

    def __init__(self):
        Node.__init__(self, 'hmi_node')
        QObject.__init__(self)
        
        self.bridge = CvBridge()
        self.auto_sort_goal_handle = None
        self.home_goal_handle = None

        # --- 1. ROS2 SUBSCRIBERS ---
        self.img_sub = self.create_subscription(Image, 'cam', self.image_callback, 10)
        self.status_sub = self.create_subscription(String, '/system/state', self.status_callback, 10)
        self.count_sub = self.create_subscription(Int32MultiArray, '/system/sorter_counts', self.count_callback, 10)
        self.error_sub = self.create_subscription(String, '/system/errors', self.error_callback, 10)

        # --- 2. ROS2 TOPIC PUBLISHERS & SERVICE CLIENTS ---
        self.speed_pub = self.create_publisher(Float32, '/robot/speed_limit', 10)
        self.reset_client = self.create_client(Trigger, '/robot/reset_error')
        self.param_client = self.create_client(SetParameters, '/vision/set_parameters')

        # --- 3. ROS2 ACTION CLIENT INITIALISATIE ---
        if HAS_CUSTOM_INTERFACES:
            self.auto_sort_client = ActionClient(self, AutoSort, 'Auto_sort')
            self.home_client = ActionClient(self, GoHome, '/robot/go_home_action')
            self.get_logger().info("✅ interfaces gevonden! Echte Action Clients voor AutoSort en GoHome zijn actief.")
        else:
            self.home_client = None
            self.get_logger().warn("⚠️ interfaces NIET gevonden. Acties draaien in TEST/MOCK modus.")

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

    # A. START KNOP (Action: Auto_sort)
    def trigger_start_sort(self):
        """Activeert automatisch sorteren via de Auto_sort Action."""
        self.get_logger().info("START knop ingedrukt!")
        
        if HAS_CUSTOM_INTERFACES:
            if self.auto_sort_goal_handle is not None:
                self.get_logger().warn("Systeem is al actief! Negeert extra start-commando.")
                return

            if not self.auto_sort_client.wait_for_server(timeout_sec=1.0):
                self.ros_error_received.emit("Auto_sort Action Server offline!")
                return
            goal_msg = AutoSort.Goal()
            goal_msg.start_request = True  
            
            send_goal_future = self.auto_sort_client.send_goal_async(
                goal_msg, 
                feedback_callback=self.auto_sort_feedback_callback
            )
            send_goal_future.add_done_callback(self.auto_sort_response_callback)
        else:
            self.get_logger().info("[TEST MODE] Action: Auto_sort Goal verzonden (start_request=True)")

    def auto_sort_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal geweigerd door de Action Server!")
            return
        self.get_logger().info("Goal succesvol geaccepteerd door de Action Server!")
        self.auto_sort_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.auto_sort_done_callback)

    def auto_sort_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.ros_state_received.emit(feedback.current_status)
        self.get_logger().info(f"[HMI Action Feedback] Status: {feedback.current_status}")

    def auto_sort_done_callback(self, future):
        """Wordt aangeroepen als de complete reeks van 10 stuks normaal is afgerond."""
        self.auto_sort_goal_handle = None
        self.get_logger().info("Volledige cyclus voltooid. Systeem staat in rust.")

    # B. STOP KNOP (Globale Action Cancel)
    def trigger_stop_sort(self):
        """Onderbreekt ALLES wat op de Action Server draait via een gecontroleerd cancel-verzoek."""
        self.get_logger().info("STOP knop ingedrukt!")
        
        if HAS_CUSTOM_INTERFACES:
            if self.auto_sort_goal_handle is not None:
                self.get_logger().info("Versturen van Cancel-verzoek naar de actieve AutoSort goal...")
                cancel_future = self.auto_sort_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self.auto_sort_cancel_response_callback)
            else:
                self.get_logger().info("Geen actieve AutoSort goal om te annuleren.")
            
            if self.home_goal_handle is not None:
                self.get_logger().info("Versturen van Cancel-verzoek naar de actieve Home goal...")
                self.home_goal_handle.cancel_goal_async()
                self.home_goal_handle = None
        else:
            self.get_logger().info("[TEST MODE] Action: cancel_all_goals_async() getriggerd.")

    def auto_sort_cancel_response_callback(self, future):
        """Wordt aangeroepen wanneer de server reageert op het cancel-verzoek."""
        self.get_logger().info("Action Server heeft het STOP-verzoek geaccepteerd. Systeem stopt na de huidige cyclus.")
        self.auto_sort_goal_handle = None

    # C. MANUAL OVERRIDE: HOME KNOP (Action: /robot/go_home_action)
    def trigger_home_action(self):
        """Stuurt de robot naar home via een Action Client."""
        self.get_logger().info("HOME knop ingedrukt!")
        
        if HAS_CUSTOM_INTERFACES and self.home_client is not None:
            if not self.home_client.wait_for_server(timeout_sec=1.0):
                self.ros_error_received.emit("GoHome Action Server offline!")
                self.get_logger().error("Kan robot niet naar home sturen: Server offline.")
                return
            
            goal_msg = GoHome.Goal()
            self.get_logger().info("Versturen van echt Home-verzoek naar de robot...")
            
            send_goal_future = self.home_client.send_goal_async(
                goal_msg,
                feedback_callback=self.home_feedback_callback
            )
            send_goal_future.add_done_callback(self.home_response_callback)
        else:
            self.get_logger().info("[TEST MODE] Action: GoHome Goal verzonden.")

    def home_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("HOME commando geweigerd door de robot-server!")
            return
        self.get_logger().info("Robot heeft het HOME commando geaccepteerd en start de beweging.")
        self.home_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self.home_done_callback)

    def home_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.ros_state_received.emit(feedback.current_status)
        self.get_logger().info(f"[HMI Home Feedback] {feedback.current_status}")

    def home_done_callback(self, future):
        """Wordt aangeroepen als de robot de home-positie succesvol heeft bereikt."""
        self.home_goal_handle = None
        self.get_logger().info("✅ Robot staat weer veilig in de HOME-positie.")

    # D. RESET ERROR KNOP (Service: /robot/reset_error)
    def trigger_reset_service(self):
        """Wist systeemfouten via een stand-alone Service Request."""
        self.get_logger().info("RESET knop ingedrukt!")
        
        if not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Reset Service offline! Kan fouten niet herstellen.")
            self.ros_error_received.emit("Reset Service is offline!")
            return

        req = Trigger.Request()
        self.get_logger().info("Versturen van echte Service Request naar /robot/reset_error...")
        self.reset_client.call_async(req)

    # E. ROBOT SNELHEID SLIDER (Topic: /robot/speed_limit)
    def publish_speed(self, speed_value):
        msg = Float32()
        msg.data = speed_value
        self.speed_pub.publish(msg)

    # F. AI PARAMETER SPINBOX (Parameter Service)
    def update_ai_parameter(self, value):
        if self.param_client.wait_for_service(timeout_sec=0.5):
            req = SetParameters.Request()
            param_val = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
            req.parameters = [Parameter(name='confidence_threshold', value=param_val)]
            self.param_client.call_async(req)