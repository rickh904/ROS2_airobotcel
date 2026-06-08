#!/usr/bin/env python3

# Naam Student:
# Studentnummer:
# Datum:
# Verklaring: Door het inleveren van dit bestand verklaar ik dat ik deze opdracht zelfstandig heb uitgevoerd en 
# dat ik geen code van anderen heb gebruikt. Tevens ga ik akkoord met de beoordeling van deze opdracht.

from threading import Thread
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Bool

from my_moveit_python import srdfGroupStates
from my_moveit_python import MovegroupHelper
import tf_transformations

# =========================================================================
# 1. VACUUM GRIPPER NODE
# =========================================================================
class VacuumGripper(Node):
    def __init__(self):
        super().__init__('vacuum_gripper')

        # Topic naam van de robotgrijper. Als dit de echte gripper is, kan deze topic direct worden gebruikt.
        self.topic_name = '/vacuum_gripper1/set_enabled'

        # Publisher voor de grijper status
        self.gripper_pub = self.create_publisher(Bool, self.topic_name, 10)
   
    def pull(self):
        msg = Bool()
        msg.data = True
        self.gripper_pub.publish(msg)
        self.get_logger().info("Gripper geactiveerd (Sluiten)")

    def release(self):
        msg = Bool()
        msg.data = False
        self.gripper_pub.publish(msg)
        self.get_logger().info("Gripper gedeactiveerd (openen)")


# =========================================================================
# 2. MANIPULATION CONTROLLER NODE
# =========================================================================
class manipulatorController(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        
        # Robot parameters (Afgestemd op jouw nieuwe envirement setup)
        prefix = ""
        self.joint_names = [
            prefix + "joint1",
            prefix + "joint2",
            prefix + "joint3",
            prefix + "joint4",
            prefix + "joint5",
            prefix + "joint6",
        ]
        
        # Gebruik de frame-namen zoals gedefinieerd in de SRDF/URDF in deze workspace
        self.base_link_name = "link_base"       # base frame zoals gebruikt in SRDF/launch
        self.end_effector_name = "vacuum_gripper1_suction_cup"     # Echte end-effector link
        self.group_name = "xarm6"                # De MoveIt planning group
        
        # Koppeling naar jouw MoveIt-configuratie package in deze workspace
        self.package_name = "manipulation_moveit_config"        
        self.srdf_file_name = "config/manipuation_environment.srdf" 

        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # MoveIt helpers
        self.group_states = srdfGroupStates(
            self.package_name, self.srdf_file_name, self.group_name
        )
        self.move_group = MovegroupHelper(
            self, self.joint_names, self.base_link_name, self.end_effector_name, self.group_name
        )

        # Grijper node initialiseren en standaard loslaten
        self.vacuum_gripper = VacuumGripper()
        self.vacuum_gripper.release()

        # Action Client voor de ManipulatorTask
        # VERVANG 'jouw_pakket_naam' bovenaan door je echte package als je de import activeert!
        # self.action_client = ActionClient(self, ManipulatorTask, 'manipulator_task')

        self.get_logger().info("manipulatorController node met Gripper is geïnitialiseerd.")

    # --- Motion primitives ------------------------------------------------
    def move_to_state(self, state_name: str):
        result, joint_values = self.group_states.get_joint_values(state_name)
        if not result:
            self.get_logger().error(f"Kan joint-waardes voor state '{state_name}' niet ophalen.")
            return
        self.get_logger().info(f"Bewegen naar state: '{state_name}'.")
        self.move_group.move_to_configuration(joint_values)

    def move_to_pose(self, translation, rotation):
        self.get_logger().info(f"Bewegen naar pose: {translation}, {rotation}")
        self.move_group.move_to_pose(translation, rotation)

    def move_to_object(self, translation, rotation, z_offset = 0.0):
        # Bereken de positie inclusief de gewenste hoogte-offset
        modified_translation = (
            translation[0],
            translation[1],
            translation[2] + z_offset
        )
        self.get_logger().info(f"Bewegen naar object met Z-offset {z_offset}: {modified_translation}")
        self.move_to_pose(modified_translation, rotation)


    # --- App sequence ----------------------------------------------------
    def execute_app(self):
        self.get_logger().info("Starten van de applicatie-sequentie...")

        # --- SIMULATIE VAN ACTION DATA ---
        # Zodra je de Action Client koppelt, komen deze data uit je `goal` binnen.
        binnenkomende_kleur = "Groen" 
        binnenkomende_translatie = (0.3, -0.2, 0.1)  # X, Y, Z coördinaten van het blokje
        binnenkomende_rotatie = (1.0, 0.0, 0.0, 0.0)    # Grijper recht naar beneden orientatie

        self.get_logger().info(f"Taak ontvangen. Kleur: {binnenkomende_kleur} op {binnenkomende_translatie}")

        # 1. Starten vanaf de veilige basispositie
        self.move_to_state("home")
        time.sleep(1.0)
    
        # 2. Ga naar de pre-grasp positie (15 cm boven het object hangen)
        self.move_to_object(binnenkomende_translatie, binnenkomende_rotatie, z_offset=0.15)
        time.sleep(1.0)
    
        # 3. Zakken naar de grasp positie (0 cm offset, direct op het blokje)
        self.move_to_object(binnenkomende_translatie, binnenkomende_rotatie, z_offset=0.0)
        time.sleep(1.0)
    
        # 4. Grijper inschakelen om het blokje vast te zuigen
        self.vacuum_gripper.pull()
        time.sleep(1.0)

        # 5. Post-grasp: Lift het blokje weer veilig 15 cm omhoog
        self.move_to_object(binnenkomende_translatie, binnenkomende_rotatie, z_offset=0.15)
        time.sleep(1.0)
        
        # 6. Terug naar home om een stabiele rotatie-as te behouden
        self.move_to_state("home")
        time.sleep(1.0)

        # 7. Sorteerlogica: Bepaal de bakcoördinaten aan de andere kant van het schot (Z=0.35)
        if binnenkomende_kleur == "Rood":
            bak_translatie = (0.4, -0.4, 0.35)
        elif binnenkomende_kleur == "Groen":
            bak_translatie = (0.4, -0.15, 0.35)
        elif binnenkomende_kleur == "Blauw":
            bak_translatie = (0.4, 0.15, 0.35)
        elif binnenkomende_kleur == "Geel":
            bak_translatie = (0.4, 0.4, 0.35)
        else:
            self.get_logger().error(f"Kleur '{binnenkomende_kleur}' onbekend. Veilig herstarten.")
            self.move_to_state("home")
            return

        # 8. Bewegen naar de geselecteerde bak (MoveIt plant de route over/om het schot)
        self.get_logger().info(f"Blokje naar de {binnenkomende_kleur}e bak brengen...")
        self.move_to_pose(bak_translatie, binnenkomende_rotatie)
        time.sleep(1.0)

        # 9. Grijper uitschakelen om het blokje te lossen
        self.vacuum_gripper.release()
        time.sleep(1.0)

        # 10. Netjes eindigen in de home-positie
        self.move_to_state("home")
        self.get_logger().info("Applicatie-sequentie succesvol afgerond.")

        
    def __del__(self):
        pass


# =========================================================================
# 3. MAIN EXECUTOR INTERFACE
# =========================================================================
def main(args=None):
    rclpy.init(args=args)

    # Maak de controller node aan
    node = manipulatorController("xarm_task_controller")

    # Multithreaded executor opstarten
    executor = MultiThreadedExecutor(num_threads=2)
    
    # CRUCIAAL: Voeg BEIDE nodes toe aan de executor zodat de publishers/TF werken
    executor.add_node(node)
    executor.add_node(node.vacuum_gripper)

    # Start de executor in de achtergrond-thread
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Geef het systeem 2 seconden om op te starten en verbinding met MoveIt te maken
    node.create_rate(0.5).sleep()

    # Start de sorteer-app
    node.execute_app()

    # Netjes afsluiten
    rclpy.shutdown()
    executor_thread.join()


if __name__ == '__main__':
    main()