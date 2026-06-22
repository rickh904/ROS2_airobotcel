#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from threading import Thread
import math
import time

from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import JointState

from my_moveit_python import srdfGroupStates, MovegroupHelper
from xarm_msgs.srv import VacuumGripperCtrl
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, MoveItErrorCodes
from interfaces.action import ManipulatorTask
import tf_transformations


class manipulatorController(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        # Robot parameters
        prefix = ""
        self.joint_names = [prefix + f"joint{i}" for i in range(1, 7)]
        self.base_link_name = "link_base"
        self.end_effector_name = "link6"
        self.group_name = "lite6"
        
        self.package_name = "my_uf_moveit_config"
        self.srdf_file_name = "config/uf_robot.srdf"

        # TF & Joint State Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.current_joint_positions = []
        self.joint_states_received = False
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        # MoveIt helpers
        self.group_states = srdfGroupStates(self.package_name, self.srdf_file_name, self.group_name)
        self.move_group = MovegroupHelper(self, self.joint_names, self.base_link_name, self.end_effector_name, self.group_name)

        # Service Clients
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Wachten op MoveIt IK service...')

        self.gripper_client = self.create_client(VacuumGripperCtrl, '/xarm/set_vacuum_gripper')
        
        # Sorteerbakjes doelen
        self.bakjes_targets = {
            "borstel": {
                "position": [0.1966, -0.1943, 0.26],
                "orientation": [math.radians(151.9), math.radians(-4.0), math.radians(1.1)]
            },
            "batterij": {
                "position": [0.3173, -0.1912, 0.26],
                "orientation": [math.radians(161.9), math.radians(-4.7), math.radians(2.5)]
            },
            "plug": {
                "position": [0.3021, -0.1055, 0.25],
                "orientation": [math.radians(-179.7), math.radians(-5.2), math.radians(-16.0)]
            },
            "bout": {
                "position": [0.1969, -0.0960, 0.25],
                "orientation": [math.radians(177.4), math.radians(-1.9), math.radians(-21.2)]
            }
        }

        self._action_server = ActionServer(
            self, ManipulatorTask, 'manipulator_task',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info("Lite6 controller succesvol opgestart.")

    def joint_cb(self, msg):
        positions = []
        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
        
        if len(positions) == 6:
            self.current_joint_positions = positions
            self.joint_states_received = True

    def wait_for_motion_done(self, target_joints, tolerance=0.08, timeout=12.0):
        start_time = time.time()
        self.get_logger().info(f"Wachten tot beweging klaar is (Tolerantie: {tolerance})...")
        time.sleep(0.4)
        
        while rclpy.ok():
            if time.time() - start_time > timeout:
                self.get_logger().warn("Timeout bereikt tijdens het wachten op beweging.")
                break
                
            if self.joint_states_received and self.current_joint_positions:
                error = sum(abs(t - c) for t, c in zip(target_joints, self.current_joint_positions))
                if error < tolerance:
                    self.get_logger().info("Doelpositie fysiek bereikt!")
                    break
            time.sleep(0.1)

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def call_gripper(self, on_state: bool):
        if self.gripper_client.wait_for_service(timeout_sec=0.5):
            request = VacuumGripperCtrl.Request()
            request.on = on_state
            self.gripper_client.call_async(request)
            time.sleep(1.0)

    def close_gripper(self):
        self.get_logger().info("Vingergrijper sluiten...")
        self.call_gripper(False)

    def open_gripper(self):
        self.get_logger().info("Vingergrijper openen...")
        self.call_gripper(True)

    def request_ik(self, translation, rotation):
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.ik_link_name = self.end_effector_name
        req.ik_request.avoid_collisions = False  
        
        req.ik_request.pose_stamped.header.frame_id = self.base_link_name
        req.ik_request.pose_stamped.pose.position.x = translation[0]
        req.ik_request.pose_stamped.pose.position.y = translation[1]
        req.ik_request.pose_stamped.pose.position.z = translation[2]
        req.ik_request.pose_stamped.pose.orientation.x = rotation[0]
        req.ik_request.pose_stamped.pose.orientation.y = rotation[1]
        req.ik_request.pose_stamped.pose.orientation.z = rotation[2]
        req.ik_request.pose_stamped.pose.orientation.w = rotation[3]

        future = self.ik_client.call_async(req)
        while rclpy.ok() and not future.done():
            time.sleep(0.05)
        return future.result()

    def move_to_pose_moveit(self, translation, rotation):
        self.get_logger().info(f"[MoveIt Planner] Traject genereren naar: {translation}")
        if hasattr(self.move_group, 'move_to_pose'):
            try:
                self.move_group.move_to_pose(translation, rotation)
                time.sleep(0.5)
                return True
            except Exception as e:
                self.get_logger().error(f"move_to_pose via MoveIt mislukt: {e}")
        return False

    def move_to_pose_forced_ik(self, translation, rotation, tolerance=0.08, timeout=12.0):
        self.get_logger().info(f"[Handmatige IK] Berekening starten voor: {translation}")
        response = self.request_ik(translation, rotation)

        if response and response.error_code.val == MoveItErrorCodes.SUCCESS:
            target_joints = list(response.solution.joint_state.position[:6])
            self.get_logger().info("Geldige joints gevonden! Uitvoeren...")
            try:
                self.move_group.move_to_configuration(target_joints)
                self.wait_for_motion_done(target_joints, tolerance=tolerance, timeout=timeout)
                return True
            except Exception as e:
                self.get_logger().error(f"Gewrichtsaansturing afgebroken: {e}")
                return False
        return False

    def move_to_state(self, state_name: str, tolerance=0.08, timeout=8.0):
        result, joint_values = self.group_states.get_joint_values(state_name)
        if not result:
            return
        self.get_logger().info(f"Moving to state '{state_name}'.")
        self.move_group.move_to_configuration(joint_values)
        self.wait_for_motion_done(joint_values, tolerance=tolerance, timeout=timeout)

    def execute_app(self):
        while rclpy.ok() and not self.joint_states_received:
            self.get_logger().info("Synchroniseren met robotposities (/joint_states)...")
            time.sleep(0.5)
        self.get_logger().info("Wachten op binnenkomende actiedoelen...")

    def execute_callback(self, goal_handle):
        self.get_logger().info('Opdracht wordt nu uitgevoerd...')
        
        feedback_msg = ManipulatorTask.Feedback()
        result = ManipulatorTask.Result()
        goal = goal_handle.request
        
        # VEILIGE HARDCODED HOOGTE: 40 mm boven de tafel om foutcode 6 te voorkomen
        final_pick_z = 0.080
        
        pre_pick_pos = [goal.position.x, goal.position.y, 0.15]
        obj_pos = [goal.position.x, goal.position.y, final_pick_z]
        obj_type = goal.object_type

        if obj_type not in self.bakjes_targets:
            goal_handle.abort()
            result.success = False
            return result

        # ROTATIE FILTEREN: Bepaal alleen de Yaw-hoek om de eigen as te draaien.
        # Roll blijft pi (recht omlaag kijken) en Pitch blijft 0.0.
        quat_in = [goal.rotation.x, goal.rotation.y, goal.rotation.z, goal.rotation.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(quat_in)

        aligned_pick_quat = tf_transformations.quaternion_from_euler(math.pi, 0.0, yaw)
        aligned_pick_rot = list(aligned_pick_quat)

        # STAP 1: Home & Open Grijper
        feedback_msg.current_state = "initializing"
        goal_handle.publish_feedback(feedback_msg)
        self.move_to_state("home", tolerance=0.08)
        self.open_gripper()

        # STAP 2: Naar 'right'
        feedback_msg.current_state = "moving_to_right"
        goal_handle.publish_feedback(feedback_msg)
        self.move_to_state("right", tolerance=0.08)

        # STAP 3A: Naar Pre-Pick via MoveIt
        feedback_msg.current_state = "moving_to_pre_pick"
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info("Beweeg naar Pre-Pick...")
        if not self.move_to_pose_moveit(pre_pick_pos, aligned_pick_rot):
            self.get_logger().error("Pre-pick beweging afgebroken.")
            goal_handle.abort()
            result.success = False
            return result

        # STAP 3B: Gecontroleerd zakken naar veilige pick-hoogte via MoveIt
        feedback_msg.current_state = "moving_to_object"
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info(f"Beweeg gecontroleerd naar harde pick-hoogte van {final_pick_z} mm...")
        
        if not self.move_to_pose_moveit(obj_pos, aligned_pick_rot):
            self.get_logger().error("Zakken afgebroken door planner!")
            goal_handle.abort()
            result.success = False
            return result
        
        time.sleep(0.5)

        # STAP 4: Grijpen
        feedback_msg.current_state = "picking_object"
        goal_handle.publish_feedback(feedback_msg)
        self.close_gripper()
        time.sleep(0.5)

        # STAP 5: Lift object recht omhoog terug naar Pre-Pick
        self.get_logger().info("Lift object omhoog...")
        if not self.move_to_pose_moveit(pre_pick_pos, aligned_pick_rot):
            self.get_logger().error("Liften mislukt. Actie afgebroken.")
            goal_handle.abort()
            result.success = False
            return result

        # STAP 6: Terug naar 'right'
        self.get_logger().info("Beweeg naar veilige tussenpositie ('right')...")
        self.move_to_state("right", tolerance=0.08)

        # STAP 7: Naar sorteerbakje via handmatige IK
        feedback_msg.current_state = "moving_to_bin"
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info(f"Start transitie naar bakje ({obj_type})...")

        bakje_xyz = self.bakjes_targets[obj_type]["position"]
        bakje_rpy = self.bakjes_targets[obj_type]["orientation"]
        bakje_quat = tf_transformations.quaternion_from_euler(bakje_rpy[0], bakje_rpy[1], bakje_rpy[2])
        
        self.move_to_pose_forced_ik(bakje_xyz, list(bakje_quat), tolerance=0.08, timeout=12.0)

        # STAP 8: Drop & Terug naar Home
        feedback_msg.current_state = "dropping_object"
        goal_handle.publish_feedback(feedback_msg)
        self.open_gripper()

        self.move_to_state("home", tolerance=0.08, timeout=10.0)

        goal_handle.succeed()
        result.success = True
        result.message = f"Object van type '{obj_type}' succesvol veilig gesorteerd!"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = manipulatorController("demo")
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    node.execute_app()

    try:
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        executor_thread.join()


if __name__ == "__main__":
    main()