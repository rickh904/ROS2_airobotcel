#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Int32MultiArray

from interfaces.action import AutoSort, GoHome

try:
    from interfaces.action import SortSpec
    HAS_SORT_SPEC = True
except ImportError:
    HAS_SORT_SPEC = False

class MainController(Node):
    def __init__(self):
        super().__init__('main_controller')
        
        self.get_logger().info("=========================================")
        self.get_logger().info("🤖 INTEGRATIE CONTROLLER ONLINE (HMI + VOICE)")
        self.get_logger().info("=========================================")

        # 1. Publishers voor de HMI GUI en State Machine
        self.status_pub = self.create_publisher(String, '/system/state', 10)
        self.count_pub = self.create_publisher(Int32MultiArray, '/system/sorter_counts', 10)

        # Interne administratie & statusvlaggen
        self.huidig_product = 1
        self.max_producten = 10
        self.counts = [0, 0, 0, 0] # [Opzetstukjes, Batterijen, Bouten, Pluggen]
        self.current_goal_handle = None
        
        # Voorkomt dat dubbelklikken of snelle opeenvolgende netwerksignalen de loop verstoren
        self.is_sorting = False
        self.current_state = "INIT"

        # =====================================================================
        # 2. ACTION SERVERS INRICHTEN (Uniform op 'AutoSort' voor HMI & Voice)
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

        # Systeem startklaar zetten
        self.update_system_state("IDLE")
        self.publish_counts()

    def update_system_state(self, state_string):
        """Publiceert de status naar het ROS2 netwerk en onthoudt deze intern."""
        self.current_state = state_string
        msg = String()
        msg.data = state_string
        self.status_pub.publish(msg)

    def publish_counts(self):
        msg = Int32MultiArray()
        msg.data = self.counts
        self.count_pub.publish(msg)

    # =========================================================================
    # HARDWARE SIMULATIE (MOCK FUNCTIONS)
    # =========================================================================
    def mock_vision_scan(self):
        time.sleep(1.0)
        mock_types = ["oral_b_head", "aaa_battery", "m6_bolt", "wall_plug"]
        index = (self.huidig_product - 1) % 4
        return mock_types[index]

    def mock_manipulator_sort(self, product_type):
        time.sleep(1.2)
        return True

    # =========================================================================
    # ROBUUSTE GOAL HOOKS (START / STOP VERWERKING)
    # =========================================================================
    def sort_goal_callback(self, goal_request):
        # 1. Spraakgestuurd STOP commando ('stop') direct afhandelen
        if hasattr(goal_request, 'command') and goal_request.command == 'stop':
            self.get_logger().warn("🗣️ Voice-commando 'STOP' ontvangen! Systeem wordt stilgelegd.")
            if self.current_goal_handle is not None:
                self.current_goal_handle.abort()
            self.is_sorting = False
            self.update_system_state("IDLE")
            return GoalResponse.REJECT

        # 2. Bescherming tegen dubbelklikken / denderen van knoppen
        if self.is_sorting or self.current_state in ["Scanning", "Calculating", "Moving_sort"]:
            self.get_logger().warn("⚠️ Systeem is al actief bezig met een reeks! Extra startverzoek genegeerd.")
            return GoalResponse.REJECT

        # 3. Valideer start-requests (HMI of Voice)
        is_voice_start = hasattr(goal_request, 'command') and goal_request.command == 'start'
        is_hmi_start = hasattr(goal_request, 'start_request') and goal_request.start_request

        if is_voice_start or is_hmi_start:
            self.get_logger().info("▶️ Sorteercyclus geaccepteerd en gestart!")
            self.is_sorting = True # Blokkeer direct opeenvolgende triggers
            if self.huidig_product > self.max_producten:
                self.huidig_product = 1
            return GoalResponse.ACCEPT
        
        return GoalResponse.REJECT

    def sort_cancel_callback(self, goal_handle):
        """Wordt getriggerd zodra de HMI op de STOP knop drukt."""
        self.get_logger().warn("🛑 HMI 'STOP' verzoek binnengekomen!")
        return CancelResponse.ACCEPT

    # =========================================================================
    # EXECUTE CALLBACKS (LOPENDE PROCESSEN)
    # =========================================================================
    def execute_sort_callback(self, goal_handle):
        self.current_goal_handle = goal_handle
        feedback_msg = AutoSort.Feedback()
        result = AutoSort.Result()

        while self.huidig_product <= self.max_producten:
            # Check of de goal tussentijds is geannuleerd of gestopt
            if not goal_handle.is_active or goal_handle.is_cancel_requested:
                break

            # STAP 1: SCANNING
            self.update_system_state("Scanning")
            feedback_msg.current_status = f"Product {self.huidig_product}/{self.max_producten}: Scannen..."
            feedback_msg.products_sorted = self.huidig_product - 1
            goal_handle.publish_feedback(feedback_msg)
            gedetecteerd_product = self.mock_vision_scan()

            # STAP 2: CALCULATING
            self.update_system_state("Calculating")
            feedback_msg.current_status = f"Referenties omrekenen voor {gedetecteerd_product}..."
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.4)

            # STAP 3: MOVING_SORT
            self.update_system_state("Moving_sort")
            feedback_msg.current_status = f"Sorteer actie uitvoeren voor {gedetecteerd_product}..."
            goal_handle.publish_feedback(feedback_msg)
            self.mock_manipulator_sort(gedetecteerd_product)

            # Tellers bijwerken
            if gedetecteerd_product == "oral_b_head":     self.counts[0] += 1
            elif gedetecteerd_product == "aaa_battery":   self.counts[1] += 1
            elif gedetecteerd_product == "m6_bolt":       self.counts[2] += 1
            elif gedetecteerd_product == "wall_plug":     self.counts[3] += 1
            self.publish_counts()

            # Afhandeling als er tijdens de actie op STOP is gedrukt
            if goal_handle.is_cancel_requested:
                self.huidig_product += 1
                self.is_sorting = False
                self.update_system_state("IDLE")
                goal_handle.canceled()
                self.current_goal_handle = None
                result.success = False
                result.message = "Geannuleerd via HMI STOP."
                return result

            self.huidig_product += 1

        # Hele cyclus doorlopen zonder onderbreking
        if goal_handle.is_active:
            goal_handle.succeed()
        
        self.is_sorting = False
        self.update_system_state("IDLE")
        self.huidig_product = 1
        self.current_goal_handle = None
        
        result.success = True
        result.message = "Sorteerreeks succesvol afgerond."
        return result

    def execute_sort_spec_callback(self, goal_handle):
        product_type = goal_handle.request.product_type
        self.get_logger().info(f"🗣️ Voice Control eist specifiek product: {product_type}")
        
        self.update_system_state("Moving_sort")
        self.mock_manipulator_sort(product_type)
        
        if product_type == "oral_b_head":   self.counts[0] += 1
        elif product_type == "aaa_battery": self.counts[1] += 1
        elif product_type == "m6_bolt":     self.counts[2] += 1
        elif product_type == "wall_plug":    self.counts[3] += 1
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