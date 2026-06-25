#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Int32MultiArray, Float32, Bool
from geometry_msgs.msg import Point, Quaternion

from interfaces.action import AutoSort, GoHome, ManipulatorTask

try:
    from interfaces.srv import CoordRef
    HAS_COORD_REF = True
except ImportError:
    CoordRef = None
    HAS_COORD_REF = False

try:
    from interfaces.srv import CoordRobot
    HAS_COORD_ROBOT = True
except ImportError:
    CoordRobot = None
    HAS_COORD_ROBOT = False

try:
    from interfaces.action import SortSpec
    HAS_SORT_SPEC = True
except ImportError:
    SortSpec = None
    HAS_SORT_SPEC = False


# =====================================================================
# CONFIGURATIE
# =====================================================================

USE_VISION_MOCK = True
MOCK_NO_PRODUCT = False

VISION_SERVICE_NAME = 'Coord_ref'
TRANSFORM_SERVICE_NAME = 'Coord_Robot'

AUTO_SORT_ACTION_NAME = 'AutoSort'
MANIPULATOR_ACTION_NAME = 'manipulator_task'

# HMI -> MainController
GO_HOME_ACTION_NAME = '/robot/go_home_action'

# MainController -> Robot.py
ROBOT_GO_HOME_ACTION_NAME = 'go_home'

SORT_SPEC_ACTION_NAME = 'sort_spec'

MOCK_VISION_DATA = (88.0, -97.0, 0.0, 45.0, "wall_plug", 0.95)

NO_PRODUCT_TIMEOUT_SEC = 15.0
SCAN_DELAY_SEC = 0.5
TRANSFORM_TIMEOUT_SEC = 5.0
VISION_TIMEOUT_SEC = 5.0
ROBOT_GOAL_ACCEPT_TIMEOUT_SEC = 10.0
ROBOT_HOME_GOAL_ACCEPT_TIMEOUT_SEC = 5.0
ROBOT_HOME_RESULT_TIMEOUT_SEC = 25.0

PRODUCT_COUNTER_INDEX = {
    "oral_b_head": 0,
    "borstel": 0,

    "aaa_battery": 1,
    "batterij": 1,

    "m6_bolt": 2,
    "bout": 2,

    "wall_plug": 3,
    "plug": 3,
}

PRODUCT_TO_ROBOT_OBJECT = {
    "oral_b_head": "borstel",
    "borstel": "borstel",

    "aaa_battery": "batterij",
    "batterij": "batterij",

    "m6_bolt": "bout",
    "bout": "bout",

    "wall_plug": "plug",
    "plug": "plug",
}


class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')

        self.cb_group = ReentrantCallbackGroup()

        self.get_logger().info("=========================================")
        self.get_logger().info("🤖 HOOFDCONTROLLER STARTUP: INITIALIZING...")
        self.get_logger().info("=========================================")

        self.status_pub = self.create_publisher(String, '/system/state', 10)
        self.count_pub = self.create_publisher(Int32MultiArray, '/system/sorter_counts', 10)
        self.error_pub = self.create_publisher(String, '/system/errors', 10)

        self.current_state = "INIT"
        self.counts = [0, 0, 0, 0]

        self.current_goal_handle = None
        self.robot_goal_handle = None
        self.robot_home_goal_handle = None

        self.is_sorting = False
        self.is_homing = False
        self.stop_requested = False

        self.confidence_threshold = 0.65
        self.last_robot_home_feedback = ""

        self.threshold_sub = self.create_subscription(
            Float32,
            '/hmi/confidence_threshold',
            self.threshold_cb,
            10,
            callback_group=self.cb_group
        )

        self.stop_sub = self.create_subscription(
            Bool,
            '/system/stop_request',
            self.stop_request_cb,
            10,
            callback_group=self.cb_group
        )

        self.update_system_state("INIT")

        # =============================================================
        # SERVICE CLIENTS
        # =============================================================

        if USE_VISION_MOCK:
            self.coord_client = None
            self.get_logger().info("ℹ️ Vision staat in MOCK-modus. CoordRef wordt niet gebruikt.")
            self.get_logger().info(
                f"🧪 MOCK DATA -> x={MOCK_VISION_DATA[0]} mm, "
                f"y={MOCK_VISION_DATA[1]} mm, "
                f"product={MOCK_VISION_DATA[4]}"
            )
        elif HAS_COORD_REF:
            self.coord_client = self.create_client(
                CoordRef,
                VISION_SERVICE_NAME,
                callback_group=self.cb_group
            )
        else:
            self.coord_client = None
            self.get_logger().error(
                "❌ USE_VISION_MOCK=False, maar CoordRef bestaat niet."
            )

        if HAS_COORD_ROBOT:
            self.transform_client = self.create_client(
                CoordRobot,
                TRANSFORM_SERVICE_NAME,
                callback_group=self.cb_group
            )
        else:
            self.transform_client = None
            self.get_logger().error(
                "❌ CoordRobot bestaat niet. Controleer interfaces/srv/CoordRobot.srv."
            )

        # =============================================================
        # ACTION CLIENTS
        # =============================================================

        self.robot_client = ActionClient(
            self,
            ManipulatorTask,
            MANIPULATOR_ACTION_NAME,
            callback_group=self.cb_group
        )

        self.robot_home_client = ActionClient(
            self,
            GoHome,
            ROBOT_GO_HOME_ACTION_NAME,
            callback_group=self.cb_group
        )

        # =============================================================
        # ACTION SERVERS
        # =============================================================

        self._sort_server = ActionServer(
            self,
            AutoSort,
            AUTO_SORT_ACTION_NAME,
            execute_callback=self.execute_sort_callback,
            goal_callback=self.sort_goal_callback,
            cancel_callback=self.sort_cancel_callback,
            callback_group=self.cb_group
        )

        self._home_server = ActionServer(
            self,
            GoHome,
            GO_HOME_ACTION_NAME,
            execute_callback=self.execute_home_callback,
            goal_callback=self.home_goal_callback,
            cancel_callback=self.home_cancel_callback,
            callback_group=self.cb_group
        )

        if HAS_SORT_SPEC:
            self._sort_spec_server = ActionServer(
                self,
                SortSpec,
                SORT_SPEC_ACTION_NAME,
                execute_callback=self.execute_sort_spec_callback,
                callback_group=self.cb_group
            )
            self.get_logger().info("✅ SortSpec action server actief.")
        else:
            self._sort_spec_server = None
            self.get_logger().warn(
                "⚠️ SortSpec bestaat nog niet in interfaces. Voice productkeuze staat uit."
            )

        self.publish_counts()
        self.check_system_communications()

    # =================================================================
    # STATUS / HMI
    # =================================================================

    def update_system_state(self, state_string: str):
        self.current_state = state_string

        msg = String()
        msg.data = state_string
        self.status_pub.publish(msg)

        self.get_logger().info(f"🔄 Systeemstatus veranderd naar: [{state_string}]")

    def publish_error(self, error_text: str):
        msg = String()
        msg.data = error_text
        self.error_pub.publish(msg)
        self.get_logger().warn(f"⚠️ {error_text}")

    def publish_counts(self):
        msg = Int32MultiArray()
        msg.data = self.counts
        self.count_pub.publish(msg)

    def threshold_cb(self, msg: Float32):
        self.confidence_threshold = float(msg.data)

        self.get_logger().info(
            f"⚙️ Confidence threshold bijgewerkt via HMI naar: "
            f"{self.confidence_threshold:.2f}"
        )

    def stop_request_cb(self, msg: Bool):
        if msg.data:
            self.get_logger().warn(
                "🛑 STOP-topic ontvangen. Huidige productcyclus wordt afgemaakt, daarna IDLE."
            )
            self.stop_requested = True

    # =================================================================
    # HELPERS
    # =================================================================

    def wait_for_future(self, future, timeout_sec: float, description: str):
        start_time = time.time()

        while rclpy.ok() and not future.done():
            if timeout_sec is not None and (time.time() - start_time) > timeout_sec:
                self.get_logger().error(f"❌ Timeout tijdens wachten op: {description}")
                return None

            time.sleep(0.01)

        try:
            return future.result()
        except Exception as e:
            self.get_logger().error(f"❌ Fout in future '{description}': {e}")
            return None

    # =================================================================
    # COMMUNICATIECHECK
    # =================================================================

    def check_system_communications(self):
        self.get_logger().info("📡 Communicatie-check gestart. Wachten op subsystemen...")

        if not USE_VISION_MOCK and self.coord_client is not None:
            while rclpy.ok() and not self.coord_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(
                    f"Wachten op AI Vision Service ('{VISION_SERVICE_NAME}')..."
                )
        elif USE_VISION_MOCK:
            self.get_logger().info("ℹ️ Vision mock actief. Vision service wordt overgeslagen.")

        if self.transform_client is not None:
            while rclpy.ok() and not self.transform_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(
                    f"Wachten op Transformatie Service ('{TRANSFORM_SERVICE_NAME}')..."
                )
        else:
            self.get_logger().warn(
                "⚠️ Geen transformatie-client actief. Fallback-coördinaten worden gebruikt."
            )

        while rclpy.ok() and not self.robot_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                f"Wachten op Robot Action Server ('{MANIPULATOR_ACTION_NAME}')..."
            )

        while rclpy.ok() and not self.robot_home_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                f"Wachten op Robot GoHome Action Server ('{ROBOT_GO_HOME_ACTION_NAME}')..."
            )

        self.get_logger().info("✅ Alle vereiste verbindingen zijn actief!")
        self.update_system_state("IDLE")

    # =================================================================
    # VISION
    # =================================================================

    def call_coord_ref_service(self):
        if USE_VISION_MOCK:
            time.sleep(0.2)

            if MOCK_NO_PRODUCT:
                return None

            return MOCK_VISION_DATA

        if self.coord_client is None:
            self.publish_error("Vision service is niet beschikbaar.")
            return None

        request = CoordRef.Request()
        future = self.coord_client.call_async(request)

        response = self.wait_for_future(
            future,
            VISION_TIMEOUT_SEC,
            "AI Vision CoordRef service"
        )

        if response is None:
            return None

        try:
            if hasattr(response, 'confidence') and float(response.confidence) <= 0.0:
                return None

            if hasattr(response, 'product_class') and str(response.product_class).strip() == "":
                return None

            return (
                response.x,
                response.y,
                response.z,
                response.rotation_z,
                response.product_class,
                response.confidence
            )

        except Exception as e:
            self.get_logger().error(f"❌ Ongeldige response van AI Vision: {e}")
            return None

    # =================================================================
    # TRANSFORMATIE
    # =================================================================

    def call_transform_service(self, cam_x, cam_y, cam_yaw):
        if self.transform_client is None:
            self.get_logger().warn(
                "⚠️ Geen transformatie-service beschikbaar. Fallback wordt gebruikt."
            )

            return (
                float(cam_x),
                float(cam_y),
                0.0,
                0.707,
                0.707,
                0.0,
                0.0
            )

        request = CoordRobot.Request()
        request.x = float(cam_x)
        request.y = float(cam_y)
        request.yaw = float(cam_yaw)

        self.get_logger().info(
            f"➡️ TRANSFORM REQUEST -> "
            f"cam_x={request.x:.3f} mm, "
            f"cam_y={request.y:.3f} mm, "
            f"yaw={request.yaw:.3f} deg"
        )

        future = self.transform_client.call_async(request)

        response = self.wait_for_future(
            future,
            TRANSFORM_TIMEOUT_SEC,
            "Coord_Robot transformatie-service"
        )

        if response is None:
            return None

        try:
            self.get_logger().info(
                f"⬅️ TRANSFORM RESPONSE -> "
                f"robot_x={response.robot_x:.3f}, "
                f"robot_y={response.robot_y:.3f}, "
                f"robot_z={response.robot_z:.3f}, "
                f"qx={response.qx:.3f}, "
                f"qy={response.qy:.3f}, "
                f"qz={response.qz:.3f}, "
                f"qw={response.qw:.3f}"
            )

            return (
                response.robot_x,
                response.robot_y,
                response.robot_z,
                response.qx,
                response.qy,
                response.qz,
                response.qw
            )

        except Exception as e:
            self.get_logger().error(f"❌ Ongeldige response van Positie_transformatie: {e}")
            return None

    # =================================================================
    # ROBOT MANIPULATOR ACTION CLIENT
    # =================================================================

    def send_manipulator_task(self, product_type, rx, ry, rz, qx, qy, qz, qw):
        if product_type not in PRODUCT_TO_ROBOT_OBJECT:
            self.get_logger().error(f"❌ Onbekend product_type: '{product_type}'")
            return False

        robot_object_type = PRODUCT_TO_ROBOT_OBJECT[product_type]

        if not self.robot_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("❌ Robot action server is niet beschikbaar.")
            return False

        goal_msg = ManipulatorTask.Goal()

        goal_msg.position = Point(
            x=float(rx),
            y=float(ry),
            z=float(rz)
        )

        goal_msg.rotation = Quaternion(
            x=float(qx),
            y=float(qy),
            z=float(qz),
            w=float(qw)
        )

        goal_msg.object_type = str(robot_object_type)

        self.get_logger().info(
            f"🤖 ROBOT GOAL -> "
            f"product='{product_type}', "
            f"robot_object='{robot_object_type}', "
            f"X={goal_msg.position.x:.3f}, "
            f"Y={goal_msg.position.y:.3f}, "
            f"Z={goal_msg.position.z:.3f}, "
            f"QX={goal_msg.rotation.x:.3f}, "
            f"QY={goal_msg.rotation.y:.3f}, "
            f"QZ={goal_msg.rotation.z:.3f}, "
            f"QW={goal_msg.rotation.w:.3f}"
        )

        send_goal_future = self.robot_client.send_goal_async(goal_msg)

        self.robot_goal_handle = self.wait_for_future(
            send_goal_future,
            ROBOT_GOAL_ACCEPT_TIMEOUT_SEC,
            "ManipulatorTask goal accept"
        )

        if self.robot_goal_handle is None or not self.robot_goal_handle.accepted:
            self.get_logger().error("❌ Robot heeft de taak geweigerd of niet geaccepteerd.")
            return False

        result_future = self.robot_goal_handle.get_result_async()

        while rclpy.ok() and not result_future.done():
            time.sleep(0.02)

        robot_result_response = result_future.result()

        if robot_result_response is None:
            self.get_logger().error("❌ Geen result teruggekregen van robot action.")
            return False

        if robot_result_response.status == 4:
            self.get_logger().info("✅ Robot action succesvol afgerond.")
            return True

        self.get_logger().error(
            f"❌ Robot action mislukt. Statuscode: {robot_result_response.status}"
        )
        return False

    # =================================================================
    # ROBOT HOME ACTION CLIENT
    # =================================================================

    def robot_home_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.last_robot_home_feedback = feedback.current_status

        self.get_logger().info(
            f"[Robot GoHome Feedback] {feedback.current_status}"
        )

    def send_robot_home_task(self, hmi_goal_handle, hmi_feedback_msg):
        if not self.robot_home_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("❌ Robot GoHome action server is niet beschikbaar.")
            return False, "Robot GoHome action server is niet beschikbaar."

        self.last_robot_home_feedback = ""

        robot_goal = GoHome.Goal()

        self.get_logger().info("➡️ GoHome goal wordt doorgestuurd naar Robot.py action '/go_home'.")

        send_goal_future = self.robot_home_client.send_goal_async(
            robot_goal,
            feedback_callback=self.robot_home_feedback_callback
        )

        self.robot_home_goal_handle = self.wait_for_future(
            send_goal_future,
            ROBOT_HOME_GOAL_ACCEPT_TIMEOUT_SEC,
            "Robot GoHome goal accept"
        )

        if self.robot_home_goal_handle is None or not self.robot_home_goal_handle.accepted:
            self.get_logger().error("❌ Robot.py heeft GoHome goal geweigerd.")
            return False, "Robot.py heeft GoHome goal geweigerd."

        result_future = self.robot_home_goal_handle.get_result_async()

        start_time = time.time()

        while rclpy.ok() and not result_future.done():
            if hmi_goal_handle.is_cancel_requested:
                self.get_logger().warn("🛑 HOME cancel ontvangen vanuit HMI.")

                cancel_future = self.robot_home_goal_handle.cancel_goal_async()
                self.wait_for_future(
                    cancel_future,
                    2.0,
                    "Robot GoHome cancel"
                )

                return False, "GoHome geannuleerd."

            if time.time() - start_time > ROBOT_HOME_RESULT_TIMEOUT_SEC:
                self.get_logger().error("❌ Timeout tijdens wachten op Robot.py GoHome result.")

                cancel_future = self.robot_home_goal_handle.cancel_goal_async()
                self.wait_for_future(
                    cancel_future,
                    2.0,
                    "Robot GoHome timeout cancel"
                )

                return False, "Timeout tijdens GoHome."

            if self.last_robot_home_feedback:
                hmi_feedback_msg.current_status = self.last_robot_home_feedback
            else:
                hmi_feedback_msg.current_status = "Robot beweegt naar HOME..."

            hmi_goal_handle.publish_feedback(hmi_feedback_msg)

            time.sleep(0.2)

        robot_result_response = result_future.result()

        if robot_result_response is None:
            self.get_logger().error("❌ Geen result ontvangen van Robot.py GoHome.")
            return False, "Geen result ontvangen van Robot.py GoHome."

        robot_result = robot_result_response.result

        if robot_result_response.status == 4 and robot_result.success:
            return True, robot_result.message

        return False, robot_result.message

    # =================================================================
    # AUTOSORT ACTION HANDLING
    # =================================================================

    def sort_goal_callback(self, goal_request):
        if self.is_sorting:
            self.get_logger().warn("⚠️ AutoSort geweigerd: systeem sorteert al.")
            return GoalResponse.REJECT

        if self.is_homing:
            self.get_logger().warn("⚠️ AutoSort geweigerd: GoHome is bezig.")
            return GoalResponse.REJECT

        if hasattr(goal_request, 'start_request') and goal_request.start_request:
            self.get_logger().info("✅ AutoSort START geaccepteerd.")
            self.is_sorting = True
            self.stop_requested = False
            return GoalResponse.ACCEPT

        self.get_logger().warn("⚠️ AutoSort goal geweigerd: start_request is niet True.")
        return GoalResponse.REJECT

    def sort_cancel_callback(self, goal_handle):
        self.get_logger().warn("🛑 STOP/cancel ontvangen door MainController.")
        self.get_logger().warn("🛑 Huidige productcyclus wordt afgemaakt; daarna IDLE.")

        self.stop_requested = True
        return CancelResponse.ACCEPT

    def execute_sort_callback(self, goal_handle):
        self.current_goal_handle = goal_handle

        feedback_msg = AutoSort.Feedback()
        result = AutoSort.Result()

        products_sorted_this_run = 0
        last_seen_time = time.time()

        self.get_logger().info("▶️ AutoSort execute-loop gestart.")

        while rclpy.ok() and self.is_sorting:
            if self.stop_requested or goal_handle.is_cancel_requested:
                self.get_logger().warn(
                    "🛑 STOP gezien vóór nieuwe scan. Geen nieuw product wordt gestart."
                )
                self.is_sorting = False
                break

            self.update_system_state("Scanning")

            feedback_msg.current_status = "Scannen naar product op stortveld..."
            feedback_msg.products_sorted = products_sorted_this_run
            goal_handle.publish_feedback(feedback_msg)

            vision_data = self.call_coord_ref_service()

            if self.stop_requested or goal_handle.is_cancel_requested:
                self.get_logger().warn(
                    "🛑 STOP gezien tijdens/na scanning. Geen robotactie gestart."
                )
                self.is_sorting = False
                break

            if vision_data is None or vision_data[0] is None:
                passed_time = time.time() - last_seen_time

                self.update_system_state("No_product")

                msg = (
                    f"Geen product gedetecteerd. Robot blijft stilstaan. "
                    f"Wachttijd: {passed_time:.1f}/{NO_PRODUCT_TIMEOUT_SEC:.1f}s"
                )

                self.publish_error(msg)

                feedback_msg.current_status = msg
                feedback_msg.products_sorted = products_sorted_this_run
                goal_handle.publish_feedback(feedback_msg)

                if passed_time >= NO_PRODUCT_TIMEOUT_SEC:
                    self.get_logger().warn(
                        "⏳ Geen product gevonden binnen timeout. AutoSort stopt."
                    )
                    self.is_sorting = False
                    break

                time.sleep(SCAN_DELAY_SEC)
                continue

            last_seen_time = time.time()

            cam_x, cam_y, cam_z, cam_yaw, detected_product, confidence = vision_data

            self.get_logger().info(
                f"👁️ PRODUCT GEZIEN -> "
                f"product={detected_product}, "
                f"cam_x={cam_x:.3f}, "
                f"cam_y={cam_y:.3f}, "
                f"cam_z={cam_z:.3f}, "
                f"yaw={cam_yaw:.3f}, "
                f"confidence={confidence:.2f}"
            )

            if confidence < self.confidence_threshold:
                msg = (
                    f"Product genegeerd: confidence {confidence:.2f} "
                    f"is lager dan threshold {self.confidence_threshold:.2f}. "
                    "Robot doet niks."
                )

                self.publish_error(msg)
                time.sleep(SCAN_DELAY_SEC)
                continue

            if detected_product not in PRODUCT_TO_ROBOT_OBJECT:
                msg = f"Onbekend product uit vision: '{detected_product}'. Robot doet niks."
                self.publish_error(msg)
                time.sleep(SCAN_DELAY_SEC)
                continue

            self.update_system_state("Calculating")

            feedback_msg.current_status = (
                f"Coördinaten transformeren voor {detected_product}..."
            )

            feedback_msg.products_sorted = products_sorted_this_run
            goal_handle.publish_feedback(feedback_msg)

            robot_data = self.call_transform_service(cam_x, cam_y, cam_yaw)

            if robot_data is None:
                self.get_logger().error("❌ Transformatie mislukt of timeout. Sorteerproces stopt.")
                self.is_sorting = False
                break

            if self.stop_requested or goal_handle.is_cancel_requested:
                self.get_logger().warn(
                    "🛑 STOP gezien na calculatie. Geen nieuwe robotactie gestart."
                )
                self.is_sorting = False
                break

            robot_x, robot_y, robot_z, rx_qx, rx_qy, rx_qz, rx_qw = robot_data

            self.update_system_state("Moving_sort")

            feedback_msg.current_status = f"Robot sorteert {detected_product}..."
            feedback_msg.products_sorted = products_sorted_this_run
            goal_handle.publish_feedback(feedback_msg)

            robot_success = self.send_manipulator_task(
                detected_product,
                robot_x,
                robot_y,
                robot_z,
                rx_qx,
                rx_qy,
                rx_qz,
                rx_qw
            )

            if not robot_success:
                self.get_logger().error("❌ Robotbeweging mislukt. Sorteerproces stopt.")
                self.is_sorting = False
                break

            counter_index = PRODUCT_COUNTER_INDEX.get(detected_product)

            if counter_index is not None:
                self.counts[counter_index] += 1
                products_sorted_this_run += 1
                self.publish_counts()

            self.get_logger().info(
                f"✅ Productcyclus afgerond. Producten gesorteerd in deze run: "
                f"{products_sorted_this_run}"
            )

            if self.stop_requested or goal_handle.is_cancel_requested:
                self.get_logger().warn(
                    "🛑 STOP verwerkt na wegleggen product. Er wordt NIET opnieuw gescand."
                )
                self.is_sorting = False
                break

            self.get_logger().info("🔁 Geen STOP ontvangen. Volgend product scannen...")

        self.update_system_state("IDLE")
        self.is_sorting = False
        self.current_goal_handle = None

        if self.stop_requested or goal_handle.is_cancel_requested:
            self.get_logger().warn("🛑 AutoSort veilig beëindigd door STOP.")

            if goal_handle.is_active:
                goal_handle.canceled()

            result.success = True
            result.message = "Sorteerproces veilig gestopt na huidige productcyclus."
            self.stop_requested = False
            return result

        if goal_handle.is_active:
            goal_handle.succeed()

        result.success = True
        result.message = "Sorteerproces beëindigd. Systeem staat in IDLE."
        self.stop_requested = False
        return result

    # =================================================================
    # HOME ACTION: HMI -> MAIN -> ROBOT.PY
    # =================================================================

    def home_goal_callback(self, goal_request):
        if self.is_sorting:
            self.get_logger().warn("⚠️ HOME geweigerd: AutoSort is actief.")
            return GoalResponse.REJECT

        if self.is_homing:
            self.get_logger().warn("⚠️ HOME geweigerd: HOME is al bezig.")
            return GoalResponse.REJECT

        self.get_logger().info("✅ HOME goal vanaf HMI geaccepteerd.")
        self.is_homing = True
        return GoalResponse.ACCEPT

    def home_cancel_callback(self, goal_handle):
        self.get_logger().warn("🛑 HOME cancel ontvangen vanaf HMI.")
        return CancelResponse.ACCEPT

    def execute_home_callback(self, goal_handle):
        self.get_logger().info("🏠 HOME-verzoek ontvangen van HMI.")

        feedback_msg = GoHome.Feedback()
        result = GoHome.Result()

        try:
            self.update_system_state("Homing_reset")

            feedback_msg.current_status = "MainController stuurt HOME naar Robot.py..."
            goal_handle.publish_feedback(feedback_msg)

            success, message = self.send_robot_home_task(
                goal_handle,
                feedback_msg
            )

            self.update_system_state("IDLE")

            if success:
                self.get_logger().info(f"✅ HOME succesvol: {message}")

                if goal_handle.is_active:
                    goal_handle.succeed()

                result.success = True
                result.message = message
                return result

            self.get_logger().error(f"❌ HOME mislukt: {message}")

            if goal_handle.is_active:
                goal_handle.abort()

            result.success = False
            result.message = message
            return result

        except Exception as e:
            self.get_logger().error(f"❌ Fout tijdens HOME in MainController: {e}")

            self.update_system_state("IDLE")

            if goal_handle.is_active:
                goal_handle.abort()

            result.success = False
            result.message = f"Fout tijdens HOME: {e}"
            return result

        finally:
            self.is_homing = False

    # =================================================================
    # SORTSPEC ACTION
    # =================================================================

    def execute_sort_spec_callback(self, goal_handle):
        product_type = goal_handle.request.product_type

        self.get_logger().info(f"🎤 SortSpec ontvangen voor product: {product_type}")

        result = SortSpec.Result()

        if product_type not in PRODUCT_TO_ROBOT_OBJECT:
            self.get_logger().error(f"❌ Onbekend SortSpec product: {product_type}")

            if goal_handle.is_active:
                goal_handle.abort()

            if hasattr(result, 'success'):
                result.success = False
            if hasattr(result, 'message'):
                result.message = f"Onbekend product: {product_type}"

            return result

        self.update_system_state("Moving_sort")

        robot_success = self.send_manipulator_task(
            product_type,
            0.20,
            0.00,
            0.00,
            0.707,
            0.707,
            0.0,
            0.0
        )

        if robot_success:
            counter_index = PRODUCT_COUNTER_INDEX.get(product_type)

            if counter_index is not None:
                self.counts[counter_index] += 1
                self.publish_counts()

            if goal_handle.is_active:
                goal_handle.succeed()

            if hasattr(result, 'success'):
                result.success = True

            if hasattr(result, 'message'):
                result.message = f"{product_type} succesvol gesorteerd."

        else:
            if goal_handle.is_active:
                goal_handle.abort()

            if hasattr(result, 'success'):
                result.success = False

            if hasattr(result, 'message'):
                result.message = f"Robot kon {product_type} niet sorteren."

        self.update_system_state("IDLE")
        return result


def main(args=None):
    rclpy.init(args=args)

    node = MainController()
    executor = MultiThreadedExecutor(num_threads=4)

    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()