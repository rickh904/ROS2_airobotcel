#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import time

# Importeer MoveIt helper utilities uit jouw workspace
from my_moveit_python import srdfGroupStates, MovegroupHelper
# Importeer de GoHome action uit jouw interfaces pakket
from interfaces.action import GoHome

class GoHomeClientNode(Node):
    def __init__(self):
        super().__init__('gohome_client_node')
        
        # --- ROBOT PARAMETERS (Gekopieerd uit jouw voorbeeld) ---
        prefix = ""
        self.joint_names = [prefix + f"joint{i}" for i in range(1, 7)]
        self.base_link_name = "link_base"
        self.end_effector_name = "link6"
        self.group_name = "lite6"
        
        self.package_name = "my_uf_moveit_config"
        self.srdf_file_name = "config/uf_robot.srdf"
        # --------------------------------------------------------

        # Initialiseer MoveIt helpers om voorgedefinieerde statussen uit te lezen en aan te sturen
        self.group_states = srdfGroupStates(self.package_name, self.srdf_file_name, self.group_name)
        self.move_group = MovegroupHelper(self, self.joint_names, self.base_link_name, self.end_effector_name, self.group_name)

        # Action Client aanmaken die luistert naar de 'go_home' server
        self._action_client = ActionClient(self, GoHome, 'go_home')
        self.get_logger().info('GoHome MoveIt Client is opgestart...')
        
        # Start direct met het uitvoeren van de thuisbeweging
        self.send_goal()

    def send_goal(self):
        self.get_logger().info('Wachten op GoHome Action Server...')
        self._action_client.wait_for_server()

        goal_msg = GoHome.Goal()
        
        self.get_logger().info('Home-doel verzenden naar de robot...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Doel geweigerd door de robotserver.')
            return

        self.get_logger().info('Doel geaccepteerd! MoveIt start de planning naar Home...')
        
        # Haal de joint-waarden voor de 'home' positie op uit het SRDF bestand
        result, joint_values = self.group_states.get_joint_values("home")
        
        if result:
            self.get_logger().info("Traject naar 'home' configureren en uitvoeren via MoveIt...")
            # Stuur de robot daadwerkelijk aan via MovegroupHelper naar de gewenste gewrichtsconfiguratie
            self.move_group.move_to_configuration(joint_values)
            self.get_logger().info('Robot is succesvol aangekomen op de Home positie!')
        else:
            self.get_logger().error("Kon de status 'home' niet vinden in het SRDF bestand!")

        # Sluit de node netjes af
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = GoHomeClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()