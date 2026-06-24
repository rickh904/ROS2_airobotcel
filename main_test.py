#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Int32MultiArray, Float32
from geometry_msgs.msg import Point, Quaternion

# Importeer de acties en de Service-interfaces
from interfaces.action import AutoSort, GoHome, ManipulatorTask

try:
    from interfaces.srv import CoordRef, CoordRobot
    HAS_SERVICES = True
except ImportError:
    HAS_SERVICES = False

try:
    from interfaces.action import SortSpec
    HAS_SORT_SPEC = True
except ImportError:
    HAS_SORT_SPEC = False


class MainController(Node):
    def __init__(self):
        # TESTVERSIE: andere node-naam zodat je ziet dat dit niet de echte main is
        super().__init__('main_controller_test')
        
        self.get_logger().info("=========================================")
        self.get_logger().info("🧪 HOOFDCONTROLLER TEST STARTUP: INITIALIZING...")
        self.get_logger().info("=========================================")

        # 1. Status direct op INIT zetten volgens specificatie
        self.status_pub = self.create_publisher(String, '/system/state', 10)
        self.count_pub = self.create_publisher(Int32MultiArray, '/system/sorter_counts', 10)
        self.update_system_state("INIT")

        # Interne administratie & statusvlaggen
        self.counts = [0, 0, 0, 0]  # [Borstel, Batterij, Bout, Plug]
        self.current_goal_handle = None
        self.robot_goal_handle = None
        self.is_sorting = False
        
        # Confidence Threshold setup
        self.confidence_threshold = 0.65
        self.threshold_sub = self.create_subscription(
            Float32,
            '/hmi/confidence_threshold',
            self.threshold_cb,
            10
        )

        # =====================================================================
        # 2. SERVICE CLIENTS & ACTION CLIENTS DEFINIËREN
        # =====================================================================
        if HAS_SERVICES:
            self.coord_client = self.create_client(CoordRef, 'Coord_ref')
            self.transform_client = self.create_client(CoordRobot, 'Coord_robot')
        else:
            self.coord_client = None
            self.transform_client = None

        self.robot_client = ActionClient(self, ManipulatorTask, 'manipulator_task')

        # =====================================================================
        # 3. TESTMODUS: COMMUNICATIECHECK OVERSLAAN
        # =====================================================================
        self.get_logger().warn("TESTMODUS: communicatiecheck overgeslagen.")
        self.get_logger().warn("Main test start zonder verplichte manipulator_task / vision / transformatie.")
        self.update_system_state("IDLE")

        # =====================================================================
        # 4. ACTION SERVERS INRICHTEN
        # =====================================================================
        self._sort_server = ActionServer(
            self, AutoSort, 'AutoSort',
            execute_callback=self.execute_sort_callback,
            goal_callback=self.sort_goal_callback,
            cancel_callback=self.sort_cancel_callback
        )
        
        self._home_server = ActionServer(
            self, GoHome, '/robot/go_home_action',
            execute_callback=self.execute_home_callback
        )

        if HAS_SORT_SPEC:
            self._sort_spec_server = ActionServer(
                self, SortSpec, 'sort_spec',
                execute_callback=self.execute_sort_spec_callback
            )

        self.publish_counts()

    def update_system_state(self, state_string):
        """Publiceert de status naar het ROS2 netwerk en onthoudt deze intern."""
        self.current_state = state_string
        msg = String()
        msg.data = state_string
        self.status_pub.publish(msg)
        self.get_logger().info(f"🔄 Systeemstatus veranderd naar: [{state_string}]")

    def publish_counts(self):
        msg = Int32MultiArray()
        msg.data = self.counts
        self.count_pub.publish(msg)

    def threshold_cb(self, msg):
        """HMI kan hiermee live de vereiste confidence aanpassen."""
        self.confidence_threshold = msg.data
        self.get_logger().info(
            f"⚙️ Confidence threshold bijgewerkt via HMI naar: {self.confidence_threshold:.2f}"
        )

    # =========================================================================
    # INIT FASE: COMMUNICATIEVERIFICATIE
    # =========================================================================
    def check_system_communications(self):
        """Originele functie blijft bestaan, maar wordt in deze testversie niet aangeroepen."""
        self.get_logger().info("📡 Communicatie-check gestart. Wachten op subsystemen...")
        
        if self.coord_client:
            while rclpy.ok() and not self.coord_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn("Wachten op AI Vision Service ('Coord_ref')...")
        
        if self.transform_client:
            while rclpy.ok() and not self.transform_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn("Wachten op Node 'Positie_transformatie' ('Coord_robot')...")
        
        while rclpy.ok() and not self.robot_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Wachten op Robot Action Server ('manipulator_task')...")

        self.get_logger().info("✅ Alle verbindingen zijn OK!")
        self.update_system_state("IDLE")

    # =========================================================================
    # ASYNCHRONE SERVICE CALLS
    # =========================================================================
    def call_coord_ref_service(self):
        """
        TESTMODUS:
        Vision wordt niet echt aangeroepen.
        Er wordt mock-data teruggegeven zodat de state machine doorloopt.
        """
        self.get_logger().warn("TESTMODUS: Coord_ref wordt niet echt aangeroepen.")
        return 0.20, -0.10, 0.05, 0.0, 1.0, "borstel", 0.85

    def call_transform_service(self, cam_x, cam_y, cam_z):
        """
        TESTMODUS:
        Positie_transformatie wordt niet echt aangeroepen.
        Camera-coördinaten worden direct als robotcoördinaten gebruikt.
        """
        self.get_logger().warn("TESTMODUS: Coord_robot wordt niet echt aangeroepen.")
        return cam_x, cam_y, cam_z

    # =========================================================================
    # ROBOT ACTIE CLIËNT
    # =========================================================================
    def send_manipulator_task(self, product_type, rx, ry, rz, qz, qw):
        """
        TESTMODUS:
        De echte manipulator_task action server wordt niet aangeroepen.
        Deze functie doet alsof de robottaak succesvol is uitgevoerd.
        """
        self.get_logger().warn("TESTMODUS: manipulator_task wordt niet echt aangeroepen.")
        self.get_logger().info(
            f"Mock robottaak ontvangen: product={product_type}, "
            f"x={rx}, y={ry}, z={rz}, qz={qz}, qw={qw}"
        )

        time.sleep(1.0)

        self.get_logger().info("TESTMODUS: mock robottaak succesvol afgerond.")
        return True

    # =========================================================================
    # ACTION GOAL HANDLING (START COMMANDO'S)
    # =========================================================================
    def sort_goal_callback(self, goal_request):
        # Stop-commando via Voice direct afvangen
        if hasattr(goal_request, 'command') and goal_request.command == 'stop':
            self.get_logger().warn(
                "🗣️ Voice-commando 'STOP' ontvangen! Huidige cyclus wordt afgemaakt, daarna IDLE."
            )
            self.is_sorting = False
            return GoalResponse.REJECT

        if self.is_sorting:
            self.get_logger().warn("⚠️ Systeem is al actief aan het sorteren!")
            return GoalResponse.REJECT

        is_voice_start = hasattr(goal_request, 'command') and goal_request.command == 'start'
        is_hmi_start = hasattr(goal_request, 'start_request') and goal_request.start_request

        if is_voice_start or is_hmi_start:
            self.get_logger().info("▶️ Sorteercyclus geaccepteerd en gestart!")
            self.is_sorting = True
            return GoalResponse.ACCEPT
        
        return GoalResponse.REJECT

    def sort_cancel_callback(self, goal_handle):
        """Wordt getriggerd bij HMI STOP knop. Systeem maakt volgens eis eerst de cyclus af."""
        self.get_logger().warn("🛑 HMI STOP ingedrukt! Systeem maakt huidige cyclus af en stopt daarna.")
        self.is_sorting = False
        return CancelResponse.ACCEPT

    # =========================================================================
    # DE DYNAMISCHE SORTEER STATE MACHINE LOOP
    # =========================================================================
    def execute_sort_callback(self, goal_handle):
        self.current_goal_handle = goal_handle
        feedback_msg = AutoSort.Feedback()
        result = AutoSort.Result()

        last_seen_time = time.time()

        while self.is_sorting:
            
            # -----------------------------------------------------------------
            # STATE: SCANNING
            # -----------------------------------------------------------------
            self.update_system_state("Scanning")
            feedback_msg.current_status = "Wachten op product op stortveld..."
            goal_handle.publish_feedback(feedback_msg)

            vision_data = self.call_coord_ref_service()
            
            if vision_data is None or vision_data[0] is None:
                passed_time = time.time() - last_seen_time
                self.get_logger().info(
                    f"⏱️ Geen producten gedetecteerd. Onbezette tijd: {passed_time:.1f}/15.0s"
                )
                
                if passed_time >= 15.0:
                    self.get_logger().warn(
                        "⏳ 15 seconden lang geen producten gezien. Sorteercyclus automatisch beëindigd."
                    )
                    self.is_sorting = False
                    break
                
                time.sleep(0.5)
                continue
            
            last_seen_time = time.time()
            cam_x, cam_y, cam_z, rot_z, rot_w, gedetecteerd_product, confidence = vision_data
            self.get_logger().info(
                f"📸 Product gescand: {gedetecteerd_product} (Confidence: {confidence:.2f})"
            )

            if confidence < self.confidence_threshold:
                self.get_logger().warn(
                    f"⚠️ Product genegeerd! Confidence ({confidence:.2f}) is lager dan threshold ({self.confidence_threshold:.2f})"
                )
                time.sleep(0.5)
                continue

            # -----------------------------------------------------------------
            # STATE: CALCULATING
            # -----------------------------------------------------------------
            self.update_system_state("Calculating")
            feedback_msg.current_status = f"Robotcoördinaten berekenen voor {gedetecteerd_product}..."
            goal_handle.publish_feedback(feedback_msg)

            robot_coords = self.call_transform_service(cam_x, cam_y, cam_z)
            if robot_coords is None:
                self.get_logger().error("Fout in coördinatentransformatie! Systeem stopt.")
                break
            robot_x, robot_y, robot_z = robot_coords

            # -----------------------------------------------------------------
            # STATE: MOVING_SORT
            # -----------------------------------------------------------------
            self.update_system_state("Moving_sort")
            feedback_msg.current_status = f"Manipulator pakt {gedetecteerd_product} op..."
            goal_handle.publish_feedback(feedback_msg)

            robot_success = self.send_manipulator_task(
                gedetecteerd_product,
                robot_x,
                robot_y,
                robot_z,
                rot_z,
                rot_w
            )
            
            if not robot_success:
                self.get_logger().error("Fysieke robotactie mislukt. Cyclus afgebroken.")
                break

            if gedetecteerd_product in ["borstel", "oral_b_head"]:
                self.counts[0] += 1
            elif gedetecteerd_product in ["batterij", "aaa_battery"]:
                self.counts[1] += 1
            elif gedetecteerd_product in ["bout", "m6_bolt"]:
                self.counts[2] += 1
            elif gedetecteerd_product in ["plug", "wall_plug"]:
                self.counts[3] += 1
            self.publish_counts()

            if goal_handle.is_cancel_requested:
                self.get_logger().info("🛑 STOP verzoek ingewilligd na het afronden van de huidige cyclus.")
                self.is_sorting = False

            # TESTMODUS:
            # Na één mock-sortering stoppen, anders blijft hij oneindig doorgaan met mock-producten.
            self.get_logger().warn("TESTMODUS: na één mock-sortering terug naar IDLE.")
            self.is_sorting = False

        # -----------------------------------------------------------------
        # STATE: TERUG NAAR IDLE
        # -----------------------------------------------------------------
        self.update_system_state("IDLE")
        self.is_sorting = False
        self.current_goal_handle = None
        
        if goal_handle.is_active:
            goal_handle.succeed()

        result.success = True
        result.message = "Sorteerproces beëindigd. Systeem staat veilig in IDLE."
        return result

    # =========================================================================
    # HANDMATIGE VOICE/HOME EXECUTIONS
    # =========================================================================
    def execute_sort_spec_callback(self, goal_handle):
        product_type = goal_handle.request.product_type
        self.update_system_state("Moving_sort")
        self.send_manipulator_task(product_type, 0.0, 0.0, 0.0, 0.0, 1.0)
        
        if product_type == "borstel":
            self.counts[0] += 1
        elif product_type == "batterij":
            self.counts[1] += 1
        elif product_type == "bout":
            self.counts[2] += 1
        elif product_type == "plug":
            self.counts[3] += 1
        self.publish_counts()

        goal_handle.succeed()
        self.update_system_state("IDLE")
        result = SortSpec.Result()
        result.success = True
        return result

    def execute_home_callback(self, goal_handle):
        self.update_system_state("Homing_reset")
        feedback_msg = GoHome.Feedback()
        feedback_msg.current_status = "Robot verplaatst naar HOME..."
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(1.5)
        goal_handle.succeed()
        self.update_system_state("IDLE")
        result = GoHome.Result()
        result.success = True
        result.message = "Robot staat veilig in HOME."
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
