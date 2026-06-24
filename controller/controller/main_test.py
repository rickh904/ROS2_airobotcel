#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy

from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Int32MultiArray, Float32
from geometry_msgs.msg import Point, Quaternion

from interfaces.action import AutoSort, ManipulatorTask
from airobot_interfaces.action import SortSpec


class MainController(Node):

    def __init__(self):
        super().__init__('main_controller_test')

        self.get_logger().info("=========================================")
        self.get_logger().info("HOOFDCONTROLLER TEST STARTUP")
        self.get_logger().info("=========================================")

        # Publishers richting HMI / systeemmonitoring
        self.status_pub = self.create_publisher(String, '/system/state', 10)
        self.count_pub = self.create_publisher(Int32MultiArray, '/system/sorter_counts', 10)

        # Interne status
        self.current_state = "INIT"
        self.counts = [0, 0, 0, 0]  # [Oral-B, AAA batterij, M6 bout, wandplug]
        self.current_goal_handle = None
        self.robot_goal_handle = None
        self.is_sorting = False

        # Productnamen moeten exact gelijk zijn aan voice_node.py
        self.valid_product_types = [
            "oral_b_head",
            "aaa_battery",
            "m6_bolt",
            "wall_plug",
        ]

        # Confidence threshold
        self.confidence_threshold = 0.65
        self.threshold_sub = self.create_subscription(
            Float32,
            '/hmi/confidence_threshold',
            self.threshold_cb,
            10
        )

        self.update_system_state("INIT")

        # In deze testversie wordt de echte manipulator niet aangeroepen.
        # Deze ActionClient blijft erin zodat de structuur lijkt op de echte main.
        self.robot_client = ActionClient(
            self,
            ManipulatorTask,
            'manipulator_task'
        )

        # Action server voor automatisch sorteren
        self._sort_server = ActionServer(
            self,
            AutoSort,
            'AutoSort',
            execute_callback=self.execute_sort_callback,
            goal_callback=self.sort_goal_callback,
            cancel_callback=self.sort_cancel_callback
        )

        # Action server voor losse productcommando's via voice
        self._sort_spec_server = ActionServer(
            self,
            SortSpec,
            'sort_spec',
            execute_callback=self.execute_sort_spec_callback
        )

        self.publish_counts()

        # Geen communicatiecheck in testmodus
        self.get_logger().warn("TESTMODUS: vision, transformatie en manipulator worden gemockt.")
        self.update_system_state("IDLE")

    # =========================================================================
    # ALGEMENE STATUSFUNCTIES
    # =========================================================================

    def update_system_state(self, state_string):
        self.current_state = state_string

        msg = String()
        msg.data = state_string
        self.status_pub.publish(msg)

        self.get_logger().info(f"🔄 Systeemstatus veranderd naar: [{state_string}]")

    def publish_counts(self):
        msg = Int32MultiArray()
        msg.data = self.counts
        self.count_pub.publish(msg)

        self.get_logger().info(
            f"📊 Tellers: Oral-B={self.counts[0]}, "
            f"AAA={self.counts[1]}, "
            f"M6 bout={self.counts[2]}, "
            f"Wandplug={self.counts[3]}"
        )

    def threshold_cb(self, msg):
        self.confidence_threshold = msg.data
        self.get_logger().info(
            f"⚙️ Confidence threshold bijgewerkt via HMI naar: {self.confidence_threshold:.2f}"
        )

    def increase_product_counter(self, product_type):
        if product_type == "oral_b_head":
            self.counts[0] += 1
        elif product_type == "aaa_battery":
            self.counts[1] += 1
        elif product_type == "m6_bolt":
            self.counts[2] += 1
        elif product_type == "wall_plug":
            self.counts[3] += 1
        else:
            self.get_logger().warn(f"Geen teller gekoppeld aan onbekend producttype: {product_type}")
            return

        self.publish_counts()

    # =========================================================================
    # MOCK VISION EN MOCK TRANSFORMATIE
    # =========================================================================

    def call_coord_ref_service(self, requested_product=None):
        """
        TESTVERSIE:
        Vision wordt niet echt aangeroepen.
        Deze functie geeft direct een geldig product terug.

        Als requested_product is meegegeven, doet de mock alsof dat product gevonden is.
        """
        if requested_product is None:
            product_type = "oral_b_head"
        else:
            product_type = requested_product

        self.get_logger().warn(f"TESTMODUS: mock vision geeft product terug: {product_type}")

        cam_x = 0.20
        cam_y = -0.10
        cam_z = 0.05
        rot_z = 0.0
        rot_w = 1.0
        confidence = 0.85

        return cam_x, cam_y, cam_z, rot_z, rot_w, product_type, confidence

    def call_transform_service(self, cam_x, cam_y, cam_z):
        """
        TESTVERSIE:
        Positie_transformatie wordt niet echt aangeroepen.
        Camera-coördinaten worden direct gebruikt als robotcoördinaten.
        """
        self.get_logger().warn("TESTMODUS: mock transformatie gebruikt camera-coördinaten als robotcoördinaten.")

        robot_x = cam_x
        robot_y = cam_y
        robot_z = cam_z

        return robot_x, robot_y, robot_z

    # =========================================================================
    # MOCK MANIPULATOR
    # =========================================================================

    def send_manipulator_task(self, product_type, rx, ry, rz, qz, qw):
        """
        TESTVERSIE:
        De echte manipulator_task action server wordt niet aangeroepen.
        Deze functie doet alleen alsof de robot succesvol beweegt.
        """
        self.get_logger().warn("TESTMODUS: manipulator_task wordt niet echt aangeroepen.")
        self.get_logger().info(
            f"🤖 Mock manipulator taak: product={product_type}, "
            f"x={rx}, y={ry}, z={rz}, qz={qz}, qw={qw}"
        )

        time.sleep(1.0)

        self.get_logger().info("✅ TESTMODUS: mock manipulator taak succesvol afgerond.")
        return True

    # =========================================================================
    # AUTOSORT ACTION: START / STOP
    # =========================================================================

    def sort_goal_callback(self, goal_request):
        if self.is_sorting:
            self.get_logger().warn("⚠️ Systeem is al actief aan het sorteren.")
            return GoalResponse.REJECT

        if hasattr(goal_request, 'start_request') and goal_request.start_request:
            self.get_logger().info("▶️ AutoSort goal geaccepteerd.")
            self.is_sorting = True
            return GoalResponse.ACCEPT

        self.get_logger().warn("AutoSort goal geweigerd: start_request is niet True.")
        return GoalResponse.REJECT

    def sort_cancel_callback(self, goal_handle):
        self.get_logger().warn("🛑 STOP ontvangen. Huidige cyclus wordt afgerond en daarna stopt AutoSort.")
        self.is_sorting = False
        return CancelResponse.ACCEPT

    def execute_sort_callback(self, goal_handle):
        self.current_goal_handle = goal_handle
        feedback_msg = AutoSort.Feedback()
        result = AutoSort.Result()

        while self.is_sorting:

            # -----------------------------------------------------------------
            # STATE: SCANNING
            # -----------------------------------------------------------------
            self.update_system_state("Scanning")
            feedback_msg.current_status = "Wachten op product op stortveld..."
            goal_handle.publish_feedback(feedback_msg)

            vision_data = self.call_coord_ref_service()

            if vision_data is None:
                self.get_logger().warn("Geen product gevonden in testmodus.")
                self.is_sorting = False
                break

            cam_x, cam_y, cam_z, rot_z, rot_w, detected_product, confidence = vision_data

            self.get_logger().info(
                f"📸 Product gedetecteerd: {detected_product}, confidence={confidence:.2f}"
            )

            if detected_product not in self.valid_product_types:
                self.get_logger().warn(f"Onbekend producttype genegeerd: {detected_product}")
                self.is_sorting = False
                break

            if confidence < self.confidence_threshold:
                self.get_logger().warn(
                    f"Confidence te laag: {confidence:.2f} < {self.confidence_threshold:.2f}"
                )
                self.is_sorting = False
                break

            # -----------------------------------------------------------------
            # STATE: CALCULATING
            # -----------------------------------------------------------------
            self.update_system_state("Calculating")
            feedback_msg.current_status = f"Robotcoördinaten berekenen voor {detected_product}..."
            goal_handle.publish_feedback(feedback_msg)

            robot_coords = self.call_transform_service(cam_x, cam_y, cam_z)

            if robot_coords is None:
                self.get_logger().error("Fout in mock positie-transformatie.")
                break

            robot_x, robot_y, robot_z = robot_coords

            # -----------------------------------------------------------------
            # STATE: MOVING_SORT
            # -----------------------------------------------------------------
            self.update_system_state("Moving_sort")
            feedback_msg.current_status = f"Manipulator sorteert {detected_product}..."
            goal_handle.publish_feedback(feedback_msg)

            robot_success = self.send_manipulator_task(
                detected_product,
                robot_x,
                robot_y,
                robot_z,
                rot_z,
                rot_w
            )

            if not robot_success:
                self.get_logger().error("Mock manipulator taak mislukt.")
                break

            self.increase_product_counter(detected_product)

            if goal_handle.is_cancel_requested:
                self.get_logger().info("🛑 STOP verwerkt na afronden huidige cyclus.")
                self.is_sorting = False
                break

            # Test-main doet één sorteercyclus en gaat daarna terug naar IDLE
            self.get_logger().info("TESTMODUS: één AutoSort-cyclus afgerond.")
            self.is_sorting = False

        self.update_system_state("IDLE")
        self.is_sorting = False
        self.current_goal_handle = None

        if goal_handle.is_active:
            goal_handle.succeed()

        result.success = True
        result.message = "AutoSort test beëindigd. Systeem staat in IDLE."
        return result

    # =========================================================================
    # SORTSPEC ACTION: LOSSE PRODUCTCOMMANDO'S VIA VOICE
    # =========================================================================

    def execute_sort_spec_callback(self, goal_handle):
        """
        Wordt aangeroepen door voice_node bij losse productcommando's.

        Voice stuurt:
        pick_oral_b_head  -> oral_b_head
        pick_aaa_battery  -> aaa_battery
        pick_m6_bolt      -> m6_bolt
        pick_wall_plug    -> wall_plug
        """
        product_type = goal_handle.request.product_type
        result = SortSpec.Result()

        self.get_logger().info("=========================================")
        self.get_logger().info(f"🗣️ Productcommando ontvangen via sort_spec: {product_type}")
        self.get_logger().info("=========================================")

        if self.is_sorting:
            self.get_logger().warn("Productcommando geweigerd: AutoSort is actief.")
            goal_handle.abort()
            result.success = False
            return result

        if product_type not in self.valid_product_types:
            self.get_logger().error(f"Onbekend producttype ontvangen via sort_spec: {product_type}")
            goal_handle.abort()
            result.success = False
            return result

        # -----------------------------------------------------------------
        # STATE: SCANNING
        # -----------------------------------------------------------------
        self.update_system_state("Scanning")
        self.get_logger().info(f"Zoeken naar specifiek product: {product_type}")

        vision_data = self.call_coord_ref_service(requested_product=product_type)

        if vision_data is None:
            self.get_logger().error(f"Specifiek product niet gevonden: {product_type}")
            goal_handle.abort()
            self.update_system_state("IDLE")
            result.success = False
            return result

        cam_x, cam_y, cam_z, rot_z, rot_w, detected_product, confidence = vision_data

        if detected_product != product_type:
            self.get_logger().error(
                f"Verkeerd product gevonden. Gevraagd={product_type}, gevonden={detected_product}"
            )
            goal_handle.abort()
            self.update_system_state("IDLE")
            result.success = False
            return result

        if confidence < self.confidence_threshold:
            self.get_logger().error(
                f"Confidence te laag voor specifiek product: {confidence:.2f}"
            )
            goal_handle.abort()
            self.update_system_state("IDLE")
            result.success = False
            return result

        # -----------------------------------------------------------------
        # STATE: CALCULATING
        # -----------------------------------------------------------------
        self.update_system_state("Calculating")
        self.get_logger().info(f"Robotcoördinaten berekenen voor specifiek product: {product_type}")

        robot_coords = self.call_transform_service(cam_x, cam_y, cam_z)

        if robot_coords is None:
            self.get_logger().error("Fout in mock positie-transformatie bij specifiek product.")
            goal_handle.abort()
            self.update_system_state("IDLE")
            result.success = False
            return result

        robot_x, robot_y, robot_z = robot_coords

        # -----------------------------------------------------------------
        # STATE: MOVING_SORT
        # -----------------------------------------------------------------
        self.update_system_state("Moving_sort")
        self.get_logger().info(f"Manipulator sorteert specifiek product: {product_type}")

        robot_success = self.send_manipulator_task(
            product_type,
            robot_x,
            robot_y,
            robot_z,
            rot_z,
            rot_w
        )

        if not robot_success:
            self.get_logger().error(f"Mock manipulator taak mislukt voor specifiek product: {product_type}")
            goal_handle.abort()
            self.update_system_state("IDLE")
            result.success = False
            return result

        self.increase_product_counter(product_type)

        goal_handle.succeed()
        self.update_system_state("IDLE")

        result.success = True
        self.get_logger().info(f"✅ Specifiek product succesvol verwerkt via sort_spec: {product_type}")
        return result


def main(args=None):
    rclpy.init(args=args)

    node = MainController()
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
