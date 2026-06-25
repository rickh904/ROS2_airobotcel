import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Int32MultiArray, Float32, Bool
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

        # Extra eigen status-tracking.
        # Hierdoor vertrouwen we niet alleen op de action goal handle.
        self.auto_sort_active = False
        self.current_system_state = "INIT"

        # =============================================================
        # 1. ROS2 SUBSCRIBERS
        # =============================================================

        self.img_sub = self.create_subscription(
            Image,
            'cam',
            self.image_callback,
            10
        )

        self.status_sub = self.create_subscription(
            String,
            '/system/state',
            self.status_callback,
            10
        )

        self.count_sub = self.create_subscription(
            Int32MultiArray,
            '/system/sorter_counts',
            self.count_callback,
            10
        )

        self.error_sub = self.create_subscription(
            String,
            '/system/errors',
            self.error_callback,
            10
        )

        # =============================================================
        # 2. ROS2 TOPIC PUBLISHERS & SERVICE CLIENTS
        # =============================================================

        self.speed_pub = self.create_publisher(
            Float32,
            '/robot/speed_limit',
            10
        )

        # BELANGRIJK:
        # Deze topic wordt altijd gepubliceerd als STOP wordt ingedrukt.
        # Hierdoor werkt STOP ook als de HMI zijn action goal handle kwijt is.
        self.stop_pub = self.create_publisher(
            Bool,
            '/system/stop_request',
            10
        )

        self.reset_client = self.create_client(
            Trigger,
            '/robot/reset_error'
        )

        self.param_client = self.create_client(
            SetParameters,
            '/vision/set_parameters'
        )

        # =============================================================
        # 3. ROS2 ACTION CLIENTS
        # =============================================================

        if HAS_CUSTOM_INTERFACES:
            self.auto_sort_client = ActionClient(
                self,
                AutoSort,
                'AutoSort'
            )

            self.home_client = ActionClient(
                self,
                GoHome,
                '/robot/go_home_action'
            )

            self.get_logger().info(
                "✅ interfaces gevonden! Echte Action Clients voor AutoSort en GoHome zijn actief."
            )
        else:
            self.auto_sort_client = None
            self.home_client = None
            self.get_logger().warn(
                "⚠️ interfaces NIET gevonden. Acties draaien in TEST/MOCK modus."
            )

    # =================================================================
    # ROS CALLBACKS
    # =================================================================

    def image_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            height, width, channel = cv_img.shape
            bytes_per_line = channel * width

            qt_img = QImage(
                cv_img.data,
                width,
                height,
                bytes_per_line,
                QImage.Format_RGB888
            ).rgbSwapped()

            self.ros_image_received.emit(qt_img)

        except Exception as e:
            self.get_logger().error(f"Fout bij camera-beeldstroom: {e}")

    def status_callback(self, msg):
        self.current_system_state = msg.data
        self.ros_state_received.emit(msg.data)

        state_upper = msg.data.upper()

        # Houd HMI-status zelf bij.
        # Hierdoor kan STOP ook logisch werken als de action result-callback raar/te vroeg komt.
        active_states = [
            "SCANNING",
            "CALCULATING",
            "MOVING_SORT",
            "MOVING",
            "SORTING",
            "RUNNING",
            "ACTIVE",
            "NO_PRODUCT"
        ]

        idle_states = [
            "IDLE",
            "INIT",
            "ERROR",
            "FAULT",
            "EMERGENCY_STOP",
            "ESTOP"
        ]

        if state_upper in active_states:
            self.auto_sort_active = True

        if state_upper in idle_states:
            self.auto_sort_active = False
            self.auto_sort_goal_handle = None

    def count_callback(self, msg):
        self.ros_counts_received.emit(list(msg.data))

    def error_callback(self, msg):
        self.ros_error_received.emit(msg.data)

    # =================================================================
    # A. START KNOP
    # =================================================================

    def trigger_start_sort(self):
        """
        Start automatisch sorteren via AutoSort action.
        """

        self.get_logger().info("START knop ingedrukt!")

        if not HAS_CUSTOM_INTERFACES:
            self.get_logger().info(
                "[TEST MODE] Action: AutoSort Goal verzonden (start_request=True)"
            )
            return

        # Niet alleen naar goal_handle kijken.
        # Als main nog actief is, mag START niet opnieuw sturen.
        if self.auto_sort_active:
            self.get_logger().warn(
                f"Systeem is al actief volgens HMI status '{self.current_system_state}'. "
                "Extra START wordt genegeerd."
            )
            return

        if self.auto_sort_goal_handle is not None:
            self.get_logger().warn(
                "Systeem heeft nog een actieve AutoSort goal handle. Extra START wordt genegeerd."
            )
            return

        if not self.auto_sort_client.wait_for_server(timeout_sec=1.0):
            self.ros_error_received.emit("AutoSort Action Server offline!")
            self.get_logger().error("AutoSort Action Server offline!")
            return

        goal_msg = AutoSort.Goal()
        goal_msg.start_request = True

        self.auto_sort_active = True

        send_goal_future = self.auto_sort_client.send_goal_async(
            goal_msg,
            feedback_callback=self.auto_sort_feedback_callback
        )

        send_goal_future.add_done_callback(self.auto_sort_response_callback)

    def auto_sort_response_callback(self, future):
        try:
            goal_handle = future.result()

            if not goal_handle.accepted:
                self.get_logger().warn("AutoSort goal geweigerd door de Action Server!")
                self.auto_sort_goal_handle = None
                self.auto_sort_active = False
                return

            self.get_logger().info("AutoSort goal succesvol geaccepteerd door de Action Server!")

            self.auto_sort_goal_handle = goal_handle
            self.auto_sort_active = True

            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.auto_sort_done_callback)

        except Exception as e:
            self.get_logger().error(f"Fout bij ontvangen AutoSort start response: {e}")
            self.auto_sort_goal_handle = None
            self.auto_sort_active = False

    def auto_sort_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        self.ros_state_received.emit(feedback.current_status)
        self.get_logger().info(
            f"[HMI Action Feedback] Status: {feedback.current_status}"
        )

        # Zolang feedback binnenkomt, is AutoSort actief.
        self.auto_sort_active = True

    def auto_sort_done_callback(self, future):
        """
        Wordt aangeroepen als AutoSort action-result binnenkomt.

        Belangrijk:
        We zetten auto_sort_active hier NIET blind op False als de main volgens
        /system/state nog bezig lijkt te zijn. Dit voorkomt dat STOP zijn handle
        te vroeg kwijtraakt.
        """

        try:
            _ = future.result()
        except Exception as e:
            self.get_logger().error(f"Fout bij AutoSort result: {e}")

        self.get_logger().info(
            "AutoSort result-callback ontvangen."
        )

        # Alleen vrijgeven als systeem ook echt IDLE lijkt.
        if self.current_system_state.upper() == "IDLE":
            self.get_logger().info(
                "Systeem staat in IDLE. AutoSort handle wordt vrijgegeven."
            )
            self.auto_sort_goal_handle = None
            self.auto_sort_active = False
        else:
            self.get_logger().warn(
                f"AutoSort result kwam binnen terwijl systeemstatus '{self.current_system_state}' is. "
                "HMI blijft STOP toestaan via /system/stop_request."
            )
            # De goal handle kan ongeldig zijn, maar STOP-topic blijft werken.
            self.auto_sort_goal_handle = None
            self.auto_sort_active = True

    # =================================================================
    # B. STOP KNOP
    # =================================================================

    def trigger_stop_sort(self):
        """
        STOP-knop.

        Nieuwe werking:
        1. Altijd /system/stop_request publiceren.
        2. Daarnaast AutoSort action cancel sturen als de goal handle nog bestaat.

        Hierdoor werkt STOP ook wanneer de HMI zijn action goal handle kwijt is.
        """

        self.get_logger().warn("STOP knop ingedrukt!")

        # 1. Altijd stop-topic sturen.
        stop_msg = Bool()
        stop_msg.data = True
        self.stop_pub.publish(stop_msg)

        self.get_logger().warn(
            "STOP-topic gepubliceerd op /system/stop_request."
        )

        if not HAS_CUSTOM_INTERFACES:
            self.get_logger().info("[TEST MODE] STOP-topic verstuurd.")
            return

        # 2. Probeer daarnaast de AutoSort action netjes te cancelen.
        if self.auto_sort_goal_handle is not None:
            self.get_logger().info(
                "Versturen van Cancel-verzoek naar de actieve AutoSort goal..."
            )

            cancel_future = self.auto_sort_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.auto_sort_cancel_response_callback)

        else:
            self.get_logger().warn(
                "HMI heeft lokaal geen actieve AutoSort goal handle. "
                "Geen probleem: STOP-topic is alsnog verstuurd naar MainController."
            )

        # 3. Home goal eventueel ook cancelen.
        if self.home_goal_handle is not None:
            self.get_logger().info(
                "Versturen van Cancel-verzoek naar de actieve Home goal..."
            )
            self.home_goal_handle.cancel_goal_async()
            self.home_goal_handle = None

    def auto_sort_cancel_response_callback(self, future):
        try:
            cancel_response = future.result()

            if len(cancel_response.goals_canceling) > 0:
                self.get_logger().info(
                    "Action Server heeft het STOP-verzoek geaccepteerd. "
                    "Systeem stopt na de huidige productcyclus."
                )
            else:
                self.get_logger().warn(
                    "Action Server heeft geen goal gecanceld. "
                    "STOP-topic is wel verstuurd, dus MainController stopt alsnog."
                )

        except Exception as e:
            self.get_logger().error(f"Fout bij STOP/cancel response: {e}")

        # Niet meteen alles vrijgeven; MainController zet straks status IDLE.
        self.auto_sort_active = True

    # =================================================================
    # C. HOME KNOP
    # =================================================================

    def trigger_home_action(self):
        """
        Stuurt de robot naar home via GoHome action.
        """

        self.get_logger().info("HOME knop ingedrukt!")

        if HAS_CUSTOM_INTERFACES and self.home_client is not None:
            if not self.home_client.wait_for_server(timeout_sec=1.0):
                self.ros_error_received.emit("GoHome Action Server offline!")
                self.get_logger().error(
                    "Kan robot niet naar home sturen: Server offline."
                )
                return

            goal_msg = GoHome.Goal()

            self.get_logger().info(
                "Versturen van echt Home-verzoek naar de robot..."
            )

            send_goal_future = self.home_client.send_goal_async(
                goal_msg,
                feedback_callback=self.home_feedback_callback
            )

            send_goal_future.add_done_callback(self.home_response_callback)

        else:
            self.get_logger().info("[TEST MODE] Action: GoHome Goal verzonden.")

    def home_response_callback(self, future):
        try:
            goal_handle = future.result()

            if not goal_handle.accepted:
                self.get_logger().warn("HOME commando geweigerd door de robot-server!")
                return

            self.get_logger().info(
                "Robot heeft het HOME commando geaccepteerd en start de beweging."
            )

            self.home_goal_handle = goal_handle
            goal_handle.get_result_async().add_done_callback(self.home_done_callback)

        except Exception as e:
            self.get_logger().error(f"Fout bij Home response: {e}")

    def home_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.ros_state_received.emit(feedback.current_status)
        self.get_logger().info(f"[HMI Home Feedback] {feedback.current_status}")

    def home_done_callback(self, future):
        self.home_goal_handle = None
        self.get_logger().info("✅ Robot staat weer veilig in de HOME-positie.")

    # =================================================================
    # D. RESET ERROR KNOP
    # =================================================================

    def trigger_reset_service(self):
        """
        Wist systeemfouten via service /robot/reset_error.
        """

        self.get_logger().info("RESET knop ingedrukt!")

        if not self.reset_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Reset Service offline! Kan fouten niet herstellen.")
            self.ros_error_received.emit("Reset Service is offline!")
            return

        req = Trigger.Request()
        self.get_logger().info(
            "Versturen van echte Service Request naar /robot/reset_error..."
        )
        self.reset_client.call_async(req)

    # =================================================================
    # E. ROBOT SNELHEID SLIDER
    # =================================================================

    def publish_speed(self, speed_value):
        msg = Float32()
        msg.data = float(speed_value)
        self.speed_pub.publish(msg)

    # =================================================================
    # F. AI PARAMETER SPINBOX
    # =================================================================

    def update_ai_parameter(self, value):
        if self.param_client.wait_for_service(timeout_sec=0.5):
            req = SetParameters.Request()

            param_val = ParameterValue(
                type=ParameterType.PARAMETER_DOUBLE,
                double_value=float(value)
            )

            req.parameters = [
                Parameter(
                    name='confidence_threshold',
                    value=param_val
                )
            ]

            self.param_client.call_async(req)