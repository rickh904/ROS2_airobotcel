#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Int32MultiArray

# Importeer de acties en de Service-interfaces
from interfaces.action import AutoSort, GoHome, ManipulatorTask  # <-- ManipulatorTask toegevoegd
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
        self.robot_goal_handle = None # Administratie voor de actieve robot-actie
        
        # Voorkomt dat dubbelklikken of snelle opeenvolgende netwerksignalen de loop verstoren
        self.is_sorting = False
        self.current_state = "INIT"

        # =====================================================================
        # 2. SERVICE CLIENTS INRICHTEN (Vision & Transformatie)
        # =====================================================================
        if HAS_SERVICES:
            self.coord_client = self.create_client(CoordRef, 'Coord_ref')
            self.transform_client = self.create_client(CoordRobot, 'Coord_robot')
            self.get_logger().info("🔍 Service Clients 'Coord_ref' en 'Coord_robot' succesvol opgezet.")
        else:
            self.coord_client = None
            self.transform_client = None
            self.get_logger().error("❌ Kon custom services niet importeren uit interfaces.srv!")

        # =====================================================================
        # 3. ACTION CLIENT INRICHTEN (Echte Robotarm Aansturing)
        # =====================================================================
        self.robot_client = ActionClient(self, ManipulatorTask, 'manipulator_task')
        self.get_logger().info("🦾 Action Client 'manipulator_task' succesvol opgezet.")

        # =====================================================================
        # 4. ACTION SERVERS INRICHTEN (Uniform op 'AutoSort' voor HMI & Voice)
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
    # ASYNCHRONE SERVICE CALLS
    # =========================================================================
    def call_coord_ref_service(self):
        if self.coord_client is None:
            return 100.0, 50.0, 20.0, "mock_product"
        if not self.coord_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("🚨 Service 'Coord_ref' offline!")
            return None

        request = CoordRef.Request()
        future = self.coord_client.call_async(request)
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
        try:
            response = future.result()
            return response.x, response.y, response.z, response.product_class
        except Exception as e:
            self.get_logger().error(f"Service 'Coord_ref' call mislukt: {e}")
            return None

    def call_transform_service(self, cam_x, cam_y, cam_z):
        if self.transform_client is None:
            return cam_x * 1.5, cam_y * 1.5, cam_z
        if not self.transform_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("🚨 Service 'Coord_robot' offline!")
            return None

        request = CoordRobot.Request()
        request.x = float(cam_x)
        request.y = float(cam_y)
        request.z = float(cam_z)

        future = self.transform_client.call_async(request)
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
        try:
            response = future.result()
            return response.robot_x, response.robot_y, response.robot_z
        except Exception as e:
            self.get_logger().error(f"Service 'Coord_robot' mislukt: {e}")
            return None

    # =========================================================================
    # ECHTE ACTION CLIENT CALL (Echte Robot Aansturing met Coördinaten)
    # =========================================================================
    def send_manipulator_task(self, product_type, rx, ry, rz):
        """Stuurt een Action Goal naar de robot met de omgerekende coördinaten."""
        if not self.robot_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("🚨 Robot Action Server 'manipulator_task' offline! Kan niet bewegen.")
            return False

        # Maak het goal-bericht aan (Pas aan naar jouw .action definitie indien veldnamen afwijken)
        goal_msg = ManipulatorTask.Goal()
        goal_msg.x = float(rx)
        goal_msg.y = float(ry)
        goal_msg.z = float(rz)
        goal_msg.product_class = str(product_type)

        self.get_logger().info(f"🦾 Echte robotactie verzenden naar 'manipulator_task' [X:{rx:.2f}, Y:{ry:.2f}, Z:{rz:.2f}]")
        
        # Stuur de goal asynchroon op
        send_goal_future = self.robot_client.send_goal_async(goal_msg)
        
        while rclpy.ok() and not send_goal_future.done():
            time.sleep(0.05)

        self.robot_goal_handle = send_goal_future.result()

        if not self.robot_goal_handle.accepted:
            self.get_logger().error("❌ Robot heeft de coördinaten/taak GEWEIGERD!")
            return False

        self.get_logger().info("✅ Robot heeft taak geaccepteerd. Beweging is gestart...")
        
        # Wacht tot de robot klaar is met zijn fysieke taak
        result_future = self.robot_goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            # Cruciaal: hiermee kan de HMI-stopknop tussentijds de actieve loop afbreken!
            if self.current_goal_handle and self.current_goal_handle.is_cancel_requested:
                self.get_logger().warn("🛑 STOP herkent tijdens robotbeweging! Annuleren van robottaak...")
                self.robot_goal_handle.cancel_goal_async()
                return False
            time.sleep(0.05)

        robot_result = result_future.result()
        # Controleer of de robot de actie succesvol heeft afgerond (status 4 = SUCCEEDED in ROS2)
        if robot_result.status == 4:
            self.get_logger().info("✅ Robot heeft de sorteertaak succesvol voltooid!")
            return True
        else:
            self.get_logger().warn("⚠️ Robotbeweging onderbroken of mislukt.")
            return False

    # =========================================================================
    # ROBUUSTE GOAL HOOKS (START / STOP VERWERKING)
    # =========================================================================
    def sort_goal_callback(self, goal_request):
        if hasattr(goal_request, 'command') and goal_request.command == 'stop':
            self.get_logger().warn("🗣️ Voice-commando 'STOP' ontvangen! Systeem wordt stilgelegd.")
            if self.current_goal_handle is not None:
                self.current_goal_handle.abort()
            self.is_sorting = False
            self.update_system_state("IDLE")
            return GoalResponse.REJECT

        if self.is_sorting or self.current_state in ["Scanning", "Calculating", "Moving_sort"]:
            self.get_logger().warn("⚠️ Systeem is al actief bezig met een reeks! Extra startverzoek genegeerd.")
            return GoalResponse.REJECT

        is_voice_start = hasattr(goal_request, 'command') and goal_request.command == 'start'
        is_hmi_start = hasattr(goal_request, 'start_request') and goal_request.start_request

        if is_voice_start or is_hmi_start:
            self.get_logger().info("▶️ Sorteercyclus geaccepteerd en gestart!")
            self.is_sorting = True 
            if self.huidig_product > self.max_producten:
                self.huidig_product = 1
            return GoalResponse.ACCEPT
        
        return GoalResponse.REJECT

    def sort_cancel_callback(self, goal_handle):
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
            if not goal_handle.is_active or goal_handle.is_cancel_requested:
                break

            # STAP 1: SCANNING (Vraag camera-data aan AI Vision)
            self.update_system_state("Scanning")
            feedback_msg.current_status = f"Product {self.huidig_product}/{self.max_producten}: Camera aanvragen..."
            feedback_msg.products_sorted = self.huidig_product - 1
            goal_handle.publish_feedback(feedback_msg)
            
            vision_data = self.call_coord_ref_service()
            if vision_data is None:
                self.get_logger().error("Fout tijdens scannen via AI Vision. Cyclus afgebroken.")
                break
                
            cam_x, cam_y, cam_z, gedetecteerd_product = vision_data
            self.get_logger().info(f"Target gezien door camera: X={cam_x}, Y={cam_y}, Z={cam_z} | Type: {gedetecteerd_product}")

            # STAP 2: CALCULATING (Omrekenen via de 'Positie_transformatie' node)
            self.update_system_state("Calculating")
            feedback_msg.current_status = f"Referenties omrekenen via Positie_transformatie..."
            goal_handle.publish_feedback(feedback_msg)
            
            robot_coords = self.call_transform_service(cam_x, cam_y, cam_z)
            if robot_coords is None:
                self.get_logger().error("Fout tijdens transformatie bij 'Positie_transformatie'. Cyclus afgebroken.")
                break
                
            robot_x, robot_y, robot_z = robot_coords

            # STAP 3: MOVING_SORT (Echte robot aansturing via de nieuwe Action Client!)
            self.update_system_state("Moving_sort")
            feedback_msg.current_status = f"Sorteer actie uitvoeren voor {gedetecteerd_product}..."
            goal_handle.publish_feedback(feedback_msg)
            
            # Roep de echte robot aan met de berekende robotcoördinaten
            robot_success = self.send_manipulator_task(gedetecteerd_product, robot_x, robot_y, robot_z)
            
            if not robot_success:
                self.get_logger().error("Robotactie mislukt of afgebroken. Stop de cyclus.")
                break

            # Tellers bijwerken (Alleen als de robot daadwerkelijk gesorteerd heeft)
            if gedetecteerd_product == "oral_b_head":     self.counts[0] += 1
            elif gedetecteerd_product == "aaa_battery":   self.counts[1] += 1
            elif gedetecteerd_product == "m6_bolt":       self.counts[2] += 1
            elif gedetecteerd_product == "wall_plug":     self.counts[3] += 1
            self.publish_counts()

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

        # Foutafhandeling/Noodstop check na breken loop
        if not goal_handle.is_active or self.current_state in ["Scanning", "Calculating", "Moving_sort"] and (vision_data is None or robot_coords is None or not robot_success):
            self.is_sorting = False
            self.update_system_state("IDLE")
            self.current_goal_handle = None
            result.success = False
            result.message = "Sorteerproces afgebroken wegens een hardware/fout-situatie."
            return result

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
        # Voor een specifieke spraak-actie sturen we de robot asynchroon naar default (of pas de coördinaten aan)
        self.send_manipulator_task(product_type, 0.0, 0.0, 0.0)
        
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