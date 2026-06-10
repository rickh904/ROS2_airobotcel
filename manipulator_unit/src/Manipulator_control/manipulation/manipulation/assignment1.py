#!/usr/bin/env python3

# Naam Student:
# Studentnummer:
# Datum:
# Verklaring: Door het inleveren van dit bestand verklaar ik dat ik deze opdracht zelfstandig heb uitgevoerd en 
# dat ik geen code van anderen heb gebruikt. Tevens ga ik akkoord met de beoordeling van deze opdracht.

from threading import Thread

import rclpy
from rclpy.executors import MultiThreadedExecutor   
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from my_moveit_python import srdfGroupStates
from my_moveit_python import MovegroupHelper
import tf_transformations


class manipulatorController(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        # Robot parameters
        prefix = ""
        self.joint_names = [
            prefix + "joint1",
            prefix + "joint2",
            prefix + "joint3",
            prefix + "joint4",
            prefix + "joint5",
            prefix + "joint6",
        ]
        self.base_link_name = "link_base"
        self.end_effector_name = "link_eef"
        self.group_name = "xarm6"
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

        # --- Create subscribers, publishers, clients, timers here ---

        self.get_logger().info("assignment1 node has been initialized.")

    # --- Create callback functions here ---

    # --- Motion primitives ------------------------------------------------
    def move_to_state(self, state_name: str):
        result, joint_values = self.group_states.get_joint_values(state_name)
        if not result:
            self.get_logger().error(f"Failed to get joint values for state '{state_name}'.")
        self.get_logger().info(f"Moving to state '{state_name}'.")
        self.move_group.move_to_configuration(joint_values)

    def move_to_pose(self, translation, rotation):
        self.get_logger().info(f"Moving to pose: {translation}, {rotation}")
        self.move_group.move_to_pose(translation, rotation)

    def move_to_tf(self, from_frame: str, to_frame: str):
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame, from_frame, rclpy.time.Time()
            )
            translation = [
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
            ]
            rotation = [
                t.transform.rotation.w,
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
            ]
            self.get_logger().info(f"Moving to transform: {from_frame} → {to_frame}")
            self.move_to_pose(translation, rotation)
        except TransformException as ex:
            self.get_logger().warn(f"Could not transform {to_frame} to {from_frame}: {ex}")

  
    # --- App sequence ----------------------------------------------------

    def execute_app(self):

        # TODO 1: Move to a specific sequence of joint states
        
       
            self.move_to_state("home")
            self.create_rate(1.0).sleep()
            self.move_to_state("left")
            self.create_rate(1.0).sleep()
            self.move_to_state("right")
            self.create_rate(1.0).sleep()
            self.move_to_state("home")

        # TODO 2: Move to a specific pose
            translation = (0.4, -0.4, 0.25)
            rotation = (1.0, 0, 0.0, 0.0)
            self.move_to_pose(translation, rotation)
       
# --------------------------------------------------------------------------
# Do not modify the main function unless necessary.
# -------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)

    # Instantiate the manipulatorController node.
    # NOTE: This must be done before creating the executor to ensure callbacks are registered correctly.
    node = manipulatorController("assignment2")

    # Create a multithreaded executor with 2 threads.
    # Allows the node to handle multiple callbacks concurrently (e.g., subscriptions, timers).
    executor = MultiThreadedExecutor(num_threads=2)

    # Add the node to the executor so it can process its callbacks.
    executor.add_node(node)

    # Start the executor in a separate background thread.
    # Keeps the ROS event loop running while allowing the main thread to execute custom logic.
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # Create a 1 Hz rate object and sleep once to allow initialization.
    # Provides time for system setup (e.g., MoveIt, TF) before running main logic.
    node.create_rate(1.0).sleep()

    # Execute the main application logic defined in the node.
    # Typically runs robot motion, computations, or control behaviors.
    node.execute_app()

    # Shutdown ROS gracefully after main logic completes.
    rclpy.shutdown()

    # Wait for the executor thread to exit cleanly before terminating the program.
    executor_thread.join()



if __name__ == "__main__":
    main()