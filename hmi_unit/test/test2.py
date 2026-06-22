#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse

# Probeer beide custom interfaces te laden
try:
    from interfaces.action import AutoSort, GoHome
except ModuleNotFoundError:
    print("❌ Fout: Pakket 'interfaces' niet gevonden of niet gecomepileerd!")
    exit(1)

class MockActionServer(Node):
    def __init__(self):
        super().__init__('mock_action_server')
        
        # --- 1. ACTION SERVER: AUTO SORT ---
        self._action_server = ActionServer(
            self,
            AutoSort,
            'AutoSort',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        # --- 2. ACTION SERVER: GO HOME ---
        self._home_server = ActionServer(
            self,
            GoHome,
            '/robot/go_home_action',
            execute_callback=self.execute_home_callback
        )
        
        # Interne variabelen voor de sorteercyclus
        self.huidig_product = 1
        self.max_producten = 10
        
        self.get_logger().info("====================================================")
        self.get_logger().info("🤖 Gedeelde Mock Action Server is ONLINE!")
        self.get_logger().info("Actieve endpoints:")
        self.get_logger().info("  ➡️  Action Server: /Auto_sort")
        self.get_logger().info("  ➡️  Action Server: /robot/go_home_action")
        self.get_logger().info("====================================================")

    # =========================================================================
    # CALLBACKS VOOR: AUTO SORT
    # =========================================================================
    def goal_callback(self, goal_request):
        if self.huidig_product > self.max_producten:
            self.get_logger().info("Vorige reeks was volbracht. Teller gereset naar Product 1.")
            self.huidig_product = 1
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("🛑 STOP knop ingedrukt op HMI! Huidige productcyclus wordt afgemaakt...")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Draait synchroon in een eigen thread dankzij MultiThreadedExecutor."""
        feedback_msg = AutoSort.Feedback()
        result = AutoSort.Result()
        
        self.get_logger().info(f"▶️ Start/Hervat reeks vanaf product {self.huidig_product}...")

        while self.huidig_product <= self.max_producten:
            
            # --- STAP 1: SCANNEN ---
            feedback_msg.current_status = f"Product {self.huidig_product}/{self.max_producten}: Scannen..."
            feedback_msg.products_sorted = self.huidig_product - 1
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1.0)
            
            # --- STAP 2: OMREKENEN ---
            feedback_msg.current_status = f"Product {self.huidig_product}/{self.max_producten}: Omrekenen..."
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1.0)
            
            # --- STAP 3: SORTEREN ---
            feedback_msg.current_status = f"Product {self.huidig_product}/{self.max_producten}: Sorteren..."
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1.0)
            
            self.get_logger().info(f"✅ Product {self.huidig_product} is volledig gesorteerd!")
            
            # --- DE CRUCIALE STOP CHECK ---
            if goal_handle.is_cancel_requested:
                self.huidig_product += 1  
                self.get_logger().warn(f"⏸️ Systeem succesvol gepauzeerd. Volgende start begint bij product {self.huidig_product}.")
                goal_handle.canceled()
                
                result.success = False
                result.message = f"Gepauzeerd na product {self.huidig_product - 1}."
                return result
            
            self.huidig_product += 1

        goal_handle.succeed()
        self.get_logger().info("✅ Alle 10 producten voltooid! Systeem reset weer naar product 1.")
        self.huidig_product = 1 
        
        result.success = True
        result.message = "Volledige reeks succesvol afgerond."
        return result

    # =========================================================================
    # CALLBACKS VOOR: GO HOME
    # =========================================================================
    def execute_home_callback(self, goal_handle):
        """Draait in een aparte thread en simuleert de homing-beweging van de arm."""
        self.get_logger().info("🏠 HOME-knop ingedrukt op HMI. Robot start homing-cyclus...")
        
        feedback_msg = GoHome.Feedback()
        result = GoHome.Result()

        # Stap 1: Start de beweging
        feedback_msg.current_status = "Robot verplaatst naar HOME..."
        goal_handle.publish_feedback(feedback_msg)
        time.sleep(1.5)  # Simuleer de bewegingstijd van de fysieke robot arm

        # Stap 2: Afronden
        goal_handle.succeed()
        self.get_logger().info("✅ Robot staat fysiek in de HOME-positie!")
        
        result.success = True
        result.message = "Home-positie succesvol bereikt."
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MockActionServer()
    
    # De MultiThreadedExecutor is essentieel om beide acties (en cancels) 
    # vlekkeloos en gelijktijdig te kunnen verwerken.
    executor = rclpy.executors.MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()