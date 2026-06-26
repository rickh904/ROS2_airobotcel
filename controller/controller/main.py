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

USE_VISION_MOCK = False
MOCK_NO_PRODUCT = False

VISION_SERVICE_NAME = '/ai_vision/coord_ref'
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

# Vision stabilisatie
VISION_SAMPLE_DURATION_SEC = 5.0
VISION_SAMPLE_DELAY_SEC = 0.5
VISION_MIN_VALID_SAMPLES = 3

# Vision geeft yaw in stappen van 15 graden
VISION_YAW_STEP_DEG = 15.0
VISION_MIN_YAW_VOTES = 3

# Plug yaw-correctie.
# Deze staat op 0.0 omdat Robot.py de yaw nu robuust correct uit rotation.x/y haalt.
PLUG_YAW_OFFSET_DEG = 0.0

# Stale object filter:
# Als vision na sorteren exact hetzelfde product op bijna dezelfde positie blijft geven,
# dan is dat waarschijnlijk een oude detectie/cache van het laatste product.
STALE_OBJECT_DISTANCE_MM = 35.0
STALE_OBJECT_YAW_DEG = 15.0
STALE_OBJECT_MAX_IGNORES = 1

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

        # Onthoudt het laatst succesvol gesorteerde product.
        # Hiermee voorkomen we dat vision dezelfde oude detectie opnieuw laat pakken.
        self.last_sorted_detection = None
        self.stale_detection_ignores = 0

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

    def normalize_yaw_180(self, yaw_deg):
        """
        Normaliseert yaw naar -90 t/m +90 graden.
        De grijper/objectrotatie is 180 graden symmetrisch.
        Dus 120 graden wordt -60 graden.
        En -120 graden wordt 60 graden.
        """

        yaw = float(yaw_deg)

        while yaw > 90.0:
            yaw -= 180.0

        while yaw <= -90.0:
            yaw += 180.0

        return yaw

    def round_yaw_to_step(self, yaw_deg, step_deg=15.0):
        yaw = self.normalize_yaw_180(yaw_deg)
        rounded = round(float(yaw) / float(step_deg)) * float(step_deg)
        return self.normalize_yaw_180(rounded)

    def yaw_difference_deg(self, yaw_a, yaw_b):
        """
        Verschil tussen twee yaws, rekening houdend met 180 graden symmetrie.
        """

        a = self.normalize_yaw_180(yaw_a)
        b = self.normalize_yaw_180(yaw_b)

        diff = a - b

        while diff > 90.0:
            diff -= 180.0

        while diff < -90.0:
            diff += 180.0

        return abs(diff)

    def median_value(self, values):
        sorted_values = sorted(float(v) for v in values)
        n = len(sorted_values)

        if n == 0:
            return 0.0

        middle = n // 2

        if n % 2 == 1:
            return sorted_values[middle]

        return (sorted_values[middle - 1] + sorted_values[middle]) / 2.0

    def remember_sorted_detection(self, product_type, cam_x, cam_y, cam_yaw):
        """
        Onthoud welk product net succesvol is gesorteerd.
        Als vision daarna exact hetzelfde blijft teruggeven, negeren we die detectie.
        """

        self.last_sorted_detection = {
            "product": str(product_type),
            "x": float(cam_x),
            "y": float(cam_y),
            "yaw": float(cam_yaw),
            "time": time.time(),
        }

        self.stale_detection_ignores = 0

        self.get_logger().info(
            f"🧠 Laatst gesorteerd onthouden: "
            f"product={product_type}, "
            f"x={float(cam_x):.1f}, "
            f"y={float(cam_y):.1f}, "
            f"yaw={float(cam_yaw):.1f}"
        )

    def is_stale_detection(self, product_type, cam_x, cam_y, cam_yaw):
        """
        Checkt of de huidige vision-detectie waarschijnlijk nog de oude detectie is
        van het product dat net weggepakt is.
        """

        if self.last_sorted_detection is None:
            return False

        last = self.last_sorted_detection

        current_robot_object = PRODUCT_TO_ROBOT_OBJECT.get(product_type, product_type)
        last_robot_object = PRODUCT_TO_ROBOT_OBJECT.get(last["product"], last["product"])

        same_product = current_robot_object == last_robot_object

        distance = math.sqrt(
            (float(cam_x) - last["x"]) ** 2 +
            (float(cam_y) - last["y"]) ** 2
        )

        yaw_diff = self.yaw_difference_deg(
            float(cam_yaw),
            last["yaw"]
        )

        is_stale = (
            same_product and
            distance <= STALE_OBJECT_DISTANCE_MM and
            yaw_diff <= STALE_OBJECT_YAW_DEG
        )

        if is_stale:
            self.get_logger().warn(
                f"♻️ Oude/stale vision-detectie genegeerd: "
                f"product={product_type}, "
                f"afstand={distance:.1f} mm, "
                f"yaw_diff={yaw_diff:.1f}°. "
                f"Dit lijkt op het product dat net al gesorteerd is."
            )

        return is_stale

    def apply_product_yaw_correction(self, product_type, cam_yaw, log=True):
        raw_yaw = float(cam_yaw)
        corrected_yaw = raw_yaw

        if product_type in ["plug", "wall_plug"]:
            corrected_yaw = raw_yaw + PLUG_YAW_OFFSET_DEG

            if log:
                self.get_logger().info(
                    f"🔧 Plug yaw-correctie: "
                    f"origineel={raw_yaw:.1f}°, "
                    f"offset={PLUG_YAW_OFFSET_DEG:+.1f}°, "
                    f"voor_normalisatie={corrected_yaw:.1f}°"
                )

        corrected_yaw = self.normalize_yaw_180(corrected_yaw)

        if log:
            self.get_logger().info(
                f"✅ Yaw na productcorrectie en normalisatie: {corrected_yaw:.1f}°"
            )

        return corrected_yaw

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

    def call_stable_coord_ref_service(self):
        valid_samples = []
        start_time = time.time()
        sample_number = 0

        self.get_logger().info(
            f"👁️ Stabiele vision-meting gestart: "
            f"{VISION_SAMPLE_DURATION_SEC:.1f}s meten, "
            f"ongeveer elke {VISION_SAMPLE_DELAY_SEC:.1f}s een sample."
        )

        while rclpy.ok() and (time.time() - start_time) < VISION_SAMPLE_DURATION_SEC:
            sample_number += 1

            vision_data = self.call_coord_ref_service()

            if vision_data is None or vision_data[0] is None:
                time.sleep(VISION_SAMPLE_DELAY_SEC)
                continue

            cam_x, cam_y, cam_z, cam_yaw, detected_product, confidence = vision_data

            if confidence < self.confidence_threshold:
                self.get_logger().warn(
                    f"👁️ Sample {sample_number} genegeerd: "
                    f"confidence={confidence:.2f} lager dan threshold={self.confidence_threshold:.2f}"
                )
                time.sleep(VISION_SAMPLE_DELAY_SEC)
                continue

            if detected_product not in PRODUCT_TO_ROBOT_OBJECT:
                self.get_logger().warn(
                    f"👁️ Sample {sample_number} genegeerd: "
                    f"onbekend product='{detected_product}'"
                )
                time.sleep(VISION_SAMPLE_DELAY_SEC)
                continue

            corrected_yaw = self.apply_product_yaw_correction(
                detected_product,
                cam_yaw,
                log=False
            )

            yaw_bucket = self.round_yaw_to_step(
                corrected_yaw,
                VISION_YAW_STEP_DEG
            )

            valid_samples.append(
                {
                    "x": float(cam_x),
                    "y": float(cam_y),
                    "z": float(cam_z),
                    "raw_yaw": float(cam_yaw),
                    "corrected_yaw": float(corrected_yaw),
                    "yaw_bucket": float(yaw_bucket),
                    "product": detected_product,
                    "confidence": float(confidence),
                }
            )

            self.get_logger().info(
                f"👁️ Sample {sample_number}: "
                f"product={detected_product}, "
                f"x={float(cam_x):.1f}, "
                f"y={float(cam_y):.1f}, "
                f"raw_yaw={float(cam_yaw):.1f}, "
                f"corrected_yaw={corrected_yaw:.1f}, "
                f"bucket={yaw_bucket:.1f}, "
                f"conf={float(confidence):.2f}"
            )

            time.sleep(VISION_SAMPLE_DELAY_SEC)

        if len(valid_samples) < VISION_MIN_VALID_SAMPLES:
            self.get_logger().warn(
                f"⚠️ Te weinig geldige vision samples: "
                f"{len(valid_samples)} samples in {VISION_SAMPLE_DURATION_SEC:.1f}s"
            )
            return None

        groups = {}

        for sample in valid_samples:
            key = (
                sample["product"],
                sample["yaw_bucket"]
            )

            if key not in groups:
                groups[key] = []

            groups[key].append(sample)

        best_key = None
        best_group = []

        for key, group in groups.items():
            if best_key is None:
                best_key = key
                best_group = group
                continue

            if len(group) > len(best_group):
                best_key = key
                best_group = group
                continue

            if len(group) == len(best_group):
                current_conf = sum(s["confidence"] for s in group) / len(group)
                best_conf = sum(s["confidence"] for s in best_group) / len(best_group)

                if current_conf > best_conf:
                    best_key = key
                    best_group = group

        if len(best_group) < VISION_MIN_YAW_VOTES:
            self.get_logger().warn(
                f"⚠️ Meest voorkomende yaw heeft maar {len(best_group)} stemmen. "
                f"Minimaal nodig: {VISION_MIN_YAW_VOTES}. Robot doet niks."
            )
            return None

        product = best_group[0]["product"]
        stable_yaw = best_group[0]["yaw_bucket"]

        stable_x = self.median_value([s["x"] for s in best_group])
        stable_y = self.median_value([s["y"] for s in best_group])
        stable_z = self.median_value([s["z"] for s in best_group])
        stable_confidence = self.median_value([s["confidence"] for s in best_group])

        self.get_logger().info(
            f"✅ Stabiele yaw gekozen: "
            f"product={product}, "
            f"yaw={stable_yaw:.1f}° met {len(best_group)}/{len(valid_samples)} stemmen | "
            f"x={stable_x:.1f}, "
            f"y={stable_y:.1f}, "
            f"confidence={stable_confidence:.2f}"
        )

        return (
            stable_x,
            stable_y,
            stable_z,
            stable_yaw,
            product,
            stable_confidence
        )

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

            # Nieuwe run, dus oude stale-detectie leegmaken.
            self.last_sorted_detection = None
            self.stale_detection_ignores = 0

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

            vision_data = self.call_stable_coord_ref_service()

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
                    f"Geen stabiel product gedetecteerd. Robot blijft stilstaan. "
                    f"Wachttijd: {passed_time:.1f}/{NO_PRODUCT_TIMEOUT_SEC:.1f}s"
                )

                self.publish_error(msg)

                feedback_msg.current_status = msg
                feedback_msg.products_sorted = products_sorted_this_run
                goal_handle.publish_feedback(feedback_msg)

                if passed_time >= NO_PRODUCT_TIMEOUT_SEC:
                    self.get_logger().warn(
                        "⏳ Geen stabiel product gevonden binnen timeout. AutoSort stopt."
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

            # =========================================================
            # NIEUW: stale/laatste-object filter
            # =========================================================
            if self.is_stale_detection(detected_product, cam_x, cam_y, cam_yaw):
                self.stale_detection_ignores += 1

                msg = (
                    f"Oude vision-detectie genegeerd "
                    f"({self.stale_detection_ignores}/{STALE_OBJECT_MAX_IGNORES}). "
                    "Waarschijnlijk is het laatste product al weggepakt."
                )

                self.publish_error(msg)

                feedback_msg.current_status = msg
                feedback_msg.products_sorted = products_sorted_this_run
                goal_handle.publish_feedback(feedback_msg)

                if self.stale_detection_ignores >= STALE_OBJECT_MAX_IGNORES:
                    self.get_logger().warn(
                        "✅ Laatste product lijkt uitgesorteerd. AutoSort stopt veilig."
                    )
                    self.is_sorting = False
                    break

                time.sleep(SCAN_DELAY_SEC)
                continue

            else:
                self.stale_detection_ignores = 0

            self.update_system_state("Calculating")

            feedback_msg.current_status = (
                f"Coördinaten transformeren voor {detected_product}..."
            )

            feedback_msg.products_sorted = products_sorted_this_run
            goal_handle.publish_feedback(feedback_msg)

            # cam_yaw is hier al product-specifiek gecorrigeerd en gestabiliseerd.
            robot_data = self.call_transform_service(
                cam_x,
                cam_y,
                cam_yaw
            )

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

            # NIEUW:
            # Na succesvolle robotactie onthouden we welke detectie net is opgepakt.
            self.remember_sorted_detection(
                detected_product,
                cam_x,
                cam_y,
                cam_yaw
            )

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