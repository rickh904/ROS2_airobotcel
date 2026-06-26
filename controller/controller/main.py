#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String, Int32MultiArray, Float32, Bool
from geometry_msgs.msg import Point, Quaternion

from interfaces.action import AutoSort, ManipulatorTask

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

# False = echte ai_vision gebruiken
# True  = test/mock zonder camera
USE_VISION_MOCK = False
MOCK_NO_PRODUCT = False

# Nieuwe vision-service
VISION_SERVICE_NAME = '/ai_vision/coord_ref'

# Transformatie-service
TRANSFORM_SERVICE_NAME = 'Coord_Robot'

AUTO_SORT_ACTION_NAME = 'AutoSort'
MANIPULATOR_ACTION_NAME = 'manipulator_task'
SORT_SPEC_ACTION_NAME = 'sort_spec'

# Format: cam_x_mm, cam_y_mm, cam_z, cam_yaw_deg, product_type, confidence
MOCK_VISION_DATA = (88.0, -97.0, 0.0, 45.0, "wall_plug", 0.95)

NO_PRODUCT_TIMEOUT_SEC = 15.0
SCAN_DELAY_SEC = 0.5
TRANSFORM_TIMEOUT_SEC = 5.0
VISION_TIMEOUT_SEC = 5.0
ROBOT_GOAL_ACCEPT_TIMEOUT_SEC = 10.0

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

        self.is_sorting = False
        self.stop_requested = False

        self.confidence_threshold = 0.65

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
                f"yaw={MOCK_VISION_DATA[3]} deg, "
                f"product={MOCK_VISION_DATA[4]}"
            )
        elif HAS_COORD_REF:
            self.coord_client = self.create_client(
                CoordRef,
                VISION_SERVICE_NAME,
                callback_group=self.cb_group
            )
            self.get_logger().info(
                f"✅ Vision client aangemaakt voor service: {VISION_SERVICE_NAME}"
            )
        else:
            self.coord_client = None
            self.get_logger().error(
                "❌ USE_VISION_MOCK=False, maar CoordRef bestaat niet. "
                "Controleer interfaces/srv/CoordRef.srv."
            )

        if HAS_COORD_ROBOT:
            self.transform_client = self.create_client(
                CoordRobot,
                TRANSFORM_SERVICE_NAME,
                callback_group=self.cb_group
            )
            self.get_logger().info(
                f"✅ Transformatie client aangemaakt voor service: {TRANSFORM_SERVICE_NAME}"
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

    def get_response_field(self, response, *field_names, default=None):
        for field_name in field_names:
            if hasattr(response, field_name):
                return getattr(response, field_name)
        return default

    def normalize_product_class(self, raw_product):
        if raw_product is None:
            return ""

        value = str(raw_product).strip().lower()
        value = value.replace("-", "_")
        value = value.replace(" ", "_")

        aliases = {
            "oral_b_head": "oral_b_head",
            "oralb_head": "oral_b_head",
            "oral_b": "oral_b_head",
            "oralb": "oral_b_head",
            "borstel": "borstel",
            "tandenborstel": "oral_b_head",
            "toothbrush": "oral_b_head",
            "toothbrush_head": "oral_b_head",

            "aaa_battery": "aaa_battery",
            "battery": "aaa_battery",
            "batterij": "batterij",
            "aaa": "aaa_battery",

            "m6_bolt": "m6_bolt",
            "bolt": "m6_bolt",
            "bout": "bout",
            "m6": "m6_bolt",

            "wall_plug": "wall_plug",
            "wallplug": "wall_plug",
            "plug": "plug",
            "muurplug": "plug",
            "wall_anchor": "wall_plug",
        }

        return aliases.get(value, value)

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
            success = self.get_response_field(
                response,
                "success",
                default=True
            )

            if success is False:
                self.get_logger().warn("👁️ Vision response: success=False, geen product.")
                return None

            raw_product = self.get_response_field(
                response,
                "class_name",
                "product_class",
                "label",
                "name",
                default=""
            )

            detected_product = self.normalize_product_class(raw_product)

            confidence = self.get_response_field(
                response,
                "confidence",
                "score",
                default=0.0
            )

            cam_x = self.get_response_field(
                response,
                "x_mm",
                "x",
                default=None
            )

            cam_y = self.get_response_field(
                response,
                "y_mm",
                "y",
                default=None
            )

            cam_z = self.get_response_field(
                response,
                "z_mm",
                "z",
                default=0.0
            )

            cam_yaw = self.get_response_field(
                response,
                "yaw_deg",
                "rotation_z",
                "yaw",
                default=0.0
            )

            if raw_product is None or str(raw_product).strip() == "":
                self.get_logger().warn("👁️ Vision gaf geen class_name/product_class terug.")
                return None

            if cam_x is None or cam_y is None:
                self.get_logger().warn("👁️ Vision gaf geen x_mm/y_mm terug.")
                return None

            cam_x = float(cam_x)
            cam_y = float(cam_y)
            cam_z = float(cam_z)
            cam_yaw = float(cam_yaw)
            confidence = float(confidence)

            if not math.isfinite(cam_x) or not math.isfinite(cam_y):
                self.get_logger().warn("👁️ Vision gaf ongeldige coördinaten terug.")
                return None

            self.get_logger().info(
                f"⬅️ VISION RESPONSE -> "
                f"raw_product='{raw_product}', "
                f"normalized_product='{detected_product}', "
                f"x_mm={cam_x:.3f}, "
                f"y_mm={cam_y:.3f}, "
                f"z={cam_z:.3f}, "
                f"yaw_deg={cam_yaw:.3f}, "
                f"confidence={confidence:.2f}"
            )

            return (
                cam_x,
                cam_y,
                cam_z,
                cam_yaw,
                detected_product,
                confidence
            )

        except Exception as e:
            self.get_logger().error(f"❌ Ongeldige response van AI Vision: {e}")
            self.get_logger().error(f"Response was: {response}")
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
    # AUTOSORT ACTION HANDLING
    # =================================================================

    def sort_goal_callback(self, goal_request):
        if self.is_sorting:
            self.get_logger().warn("⚠️ AutoSort geweigerd: systeem sorteert al.")
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
                f"cam_x_mm={cam_x:.3f}, "
                f"cam_y_mm={cam_y:.3f}, "
                f"cam_z={cam_z:.3f}, "
                f"yaw_deg={cam_yaw:.3f}, "
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
                msg = (
                    f"Onbekend product uit vision: '{detected_product}'. "
                    "Robot doet niks. Voeg dit label toe aan PRODUCT_TO_ROBOT_OBJECT."
                )
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