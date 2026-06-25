#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
from threading import Thread, Lock

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import JointState

from my_moveit_python import srdfGroupStates, MovegroupHelper
from xarm_msgs.srv import VacuumGripperCtrl
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import MoveItErrorCodes

from interfaces.action import ManipulatorTask, GoHome

import tf_transformations


class manipulatorController(Node):
    def __init__(self, node_name):
        super().__init__(node_name)

        self.cb_group = ReentrantCallbackGroup()

        # =============================================================
        # ROBOT PARAMETERS
        # =============================================================

        prefix = ""
        self.joint_names = [prefix + f"joint{i}" for i in range(1, 7)]
        self.base_link_name = "link_base"
        self.end_effector_name = "link6"
        self.group_name = "lite6"

        self.package_name = "my_uf_moveit_config"
        self.srdf_file_name = "config/uf_robot.srdf"

        # Voorkomt dat HOME en sorteren tegelijk MoveIt gebruiken.
        self.robot_busy = False
        self.active_task = "idle"
        self.busy_lock = Lock()

        # =============================================================
        # TF & JOINT STATES
        # =============================================================

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_joint_positions = []
        self.joint_states_received = False

        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_cb,
            10,
            callback_group=self.cb_group
        )

        # =============================================================
        # MOVEIT HELPERS
        # =============================================================

        self.group_states = srdfGroupStates(
            self.package_name,
            self.srdf_file_name,
            self.group_name
        )

        self.move_group = MovegroupHelper(
            self,
            self.joint_names,
            self.base_link_name,
            self.end_effector_name,
            self.group_name
        )

        # =============================================================
        # SERVICE CLIENTS
        # =============================================================

        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik',
            callback_group=self.cb_group
        )

        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Wachten op MoveIt IK service...')

        self.gripper_client = self.create_client(
            VacuumGripperCtrl,
            '/xarm/set_vacuum_gripper',
            callback_group=self.cb_group
        )

        # =============================================================
        # SORTEERBAKJES
        # =============================================================

        self.bakjes_targets = {
            "borstel": {
                "position": [0.1966, -0.1943, 0.26],
                "orientation": [
                    math.radians(151.9),
                    math.radians(-4.0),
                    math.radians(1.1)
                ]
            },
            "batterij": {
                "position": [0.3173, -0.1912, 0.26],
                "orientation": [
                    math.radians(161.9),
                    math.radians(-4.7),
                    math.radians(2.5)
                ]
            },
            "plug": {
                "position": [0.3021, -0.1055, 0.26],
                "orientation": [
                    math.radians(-179.7),
                    math.radians(-5.2),
                    math.radians(-16.0)
                ]
            },
            "bout": {
                "position": [0.1969, -0.0960, 0.26],
                "orientation": [
                    math.radians(177.4),
                    math.radians(-1.9),
                    math.radians(-21.2)
                ]
            }
        }

        # =============================================================
        # ACTION SERVERS
        # =============================================================

        self._manipulator_action_server = ActionServer(
            self,
            ManipulatorTask,
            'manipulator_task',
            execute_callback=self.execute_manipulator_callback,
            goal_callback=self.manipulator_goal_callback,
            cancel_callback=self.manipulator_cancel_callback,
            callback_group=self.cb_group
        )

        self._go_home_action_server = ActionServer(
            self,
            GoHome,
            'go_home',
            execute_callback=self.execute_go_home_callback,
            goal_callback=self.go_home_goal_callback,
            cancel_callback=self.go_home_cancel_callback,
            callback_group=self.cb_group
        )

        self.get_logger().info("✅ Lite6 Robot.py controller succesvol opgestart.")
        self.get_logger().info("✅ Action server actief: /manipulator_task")
        self.get_logger().info("✅ Action server actief: /go_home")

    # =================================================================
    # BUSY LOCK
    # =================================================================

    def acquire_robot(self, task_name: str) -> bool:
        with self.busy_lock:
            if self.robot_busy:
                self.get_logger().warn(
                    f"⚠️ Robot is al bezig met '{self.active_task}'. "
                    f"Nieuwe taak '{task_name}' wordt geweigerd."
                )
                return False

            self.robot_busy = True
            self.active_task = task_name
            return True

    def release_robot(self):
        with self.busy_lock:
            self.robot_busy = False
            self.active_task = "idle"

    # =================================================================
    # JOINT STATES
    # =================================================================

    def joint_cb(self, msg):
        positions = []

        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])

        if len(positions) == 6:
            self.current_joint_positions = positions
            self.joint_states_received = True

    def wait_for_joint_states(self, timeout=5.0):
        start_time = time.time()

        while rclpy.ok() and not self.joint_states_received:
            if time.time() - start_time > timeout:
                self.get_logger().error("❌ Timeout: geen /joint_states ontvangen.")
                return False

            self.get_logger().info("Synchroniseren met robotposities (/joint_states)...")
            time.sleep(0.2)

        return True

    def log_joint_diagnostics(self):
        if not self.current_joint_positions or len(self.current_joint_positions) != 6:
            return

        joints_text = ", ".join(
            [
                f"joint{i + 1}={value:.3f}"
                for i, value in enumerate(self.current_joint_positions)
            ]
        )

        self.get_logger().info(f"Huidige joints: {joints_text}")

        joint4 = self.current_joint_positions[3]

        if joint4 < -3.11 or joint4 > 3.11:
            self.get_logger().error(
                f"❌ Diagnose: joint4={joint4:.3f} rad staat buiten MoveIt-limiet "
                "[-3.11, 3.11]. HOME kan daardoor falen."
            )

    # =================================================================
    # BEWEGING HELPERS
    # =================================================================

    def wait_for_motion_done(self, target_joints, tolerance=0.08, timeout=12.0):
        start_time = time.time()

        self.get_logger().info(
            f"Wachten tot beweging klaar is. Tolerantie: {tolerance}, timeout: {timeout}s"
        )

        time.sleep(0.4)

        while rclpy.ok():
            if time.time() - start_time > timeout:
                self.get_logger().warn("⚠️ Timeout bereikt tijdens wachten op beweging.")
                return False

            if self.joint_states_received and self.current_joint_positions:
                error = sum(
                    abs(target - current)
                    for target, current in zip(target_joints, self.current_joint_positions)
                )

                if error < tolerance:
                    self.get_logger().info("✅ Doelpositie fysiek bereikt!")
                    return True

            time.sleep(0.1)

        return False

    def move_to_state(self, state_name: str, tolerance=0.08, timeout=10.0):
        result, joint_values = self.group_states.get_joint_values(state_name)

        if not result:
            self.get_logger().error(
                f"❌ State '{state_name}' niet gevonden in SRDF."
            )
            return False

        self.get_logger().info(f"➡️ Moving to state '{state_name}'.")
        self.get_logger().info(
            "Target joints: "
            + ", ".join([f"{joint:.3f}" for joint in joint_values])
        )

        self.log_joint_diagnostics()

        try:
            self.move_group.move_to_configuration(joint_values)
        except Exception as e:
            self.get_logger().error(
                f"❌ move_to_configuration('{state_name}') mislukt: {e}"
            )
            return False

        reached = self.wait_for_motion_done(
            joint_values,
            tolerance=tolerance,
            timeout=timeout
        )

        if not reached:
            self.get_logger().error(
                f"❌ State '{state_name}' niet bevestigd via /joint_states."
            )
            return False

        return True

    # =================================================================
    # GRIJPER
    # =================================================================

    def call_gripper(self, on_state: bool):
        if self.gripper_client.wait_for_service(timeout_sec=0.5):
            request = VacuumGripperCtrl.Request()
            request.on = on_state
            self.gripper_client.call_async(request)
            time.sleep(1.0)
        else:
            self.get_logger().warn("⚠️ Gripper service niet beschikbaar.")

    def close_gripper(self):
        self.get_logger().info("Vingergrijper sluiten...")
        self.call_gripper(False)

    def open_gripper(self):
        self.get_logger().info("Vingergrijper openen...")
        self.call_gripper(True)

    # =================================================================
    # IK / POSE MOVE
    # =================================================================

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
        self.get_logger().info(
            f"[MoveIt Planner] Traject genereren naar: {translation}"
        )

        if hasattr(self.move_group, 'move_to_pose'):
            try:
                self.move_group.move_to_pose(translation, rotation)
                time.sleep(0.5)
                return True
            except Exception as e:
                self.get_logger().error(
                    f"move_to_pose via MoveIt mislukt: {e}"
                )

        return False

    def move_to_pose_forced_ik(self, translation, rotation, tolerance=0.08, timeout=12.0):
        self.get_logger().info(
            f"[Handmatige IK] Berekening starten voor: {translation}"
        )

        response = self.request_ik(translation, rotation)

        if response and response.error_code.val == MoveItErrorCodes.SUCCESS:
            target_joints = list(response.solution.joint_state.position[:6])
            self.get_logger().info("Geldige joints gevonden! Uitvoeren...")

            try:
                self.move_group.move_to_configuration(target_joints)

                return self.wait_for_motion_done(
                    target_joints,
                    tolerance=tolerance,
                    timeout=timeout
                )

            except Exception as e:
                self.get_logger().error(
                    f"Gewrichtsaansturing afgebroken: {e}"
                )
                return False

        self.get_logger().error("❌ Geen geldige IK-oplossing gevonden.")
        return False

    # =================================================================
    # MANIPULATOR TASK ACTION
    # =================================================================

    def manipulator_goal_callback(self, goal_request):
        if not self.acquire_robot("manipulator_task"):
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def manipulator_cancel_callback(self, goal_handle):
        self.get_logger().warn(
            "🛑 Cancel ontvangen voor manipulator_task. "
            "Huidige beweging wordt niet hard onderbroken."
        )
        return CancelResponse.ACCEPT

    def execute_manipulator_callback(self, goal_handle):
        self.get_logger().info('Opdracht wordt nu uitgevoerd...')

        feedback_msg = ManipulatorTask.Feedback()
        result = ManipulatorTask.Result()
        goal = goal_handle.request

        try:
            if not self.wait_for_joint_states(timeout=5.0):
                goal_handle.abort()
                result.success = False
                result.message = "Geen joint states ontvangen."
                return result

            final_pick_z = 0.08

            pre_pick_pos = [goal.position.x, goal.position.y, 0.15]
            obj_pos = [goal.position.x, goal.position.y, final_pick_z]
            obj_type = goal.object_type

            if obj_type not in self.bakjes_targets:
                self.get_logger().error(f"❌ Onbekend object_type: {obj_type}")
                goal_handle.abort()
                result.success = False
                result.message = f"Onbekend object_type: {obj_type}"
                return result

            quat_in = [
                goal.rotation.x,
                goal.rotation.y,
                goal.rotation.z,
                goal.rotation.w
            ]

            _, _, yaw = tf_transformations.euler_from_quaternion(quat_in)

            aligned_pick_quat = tf_transformations.quaternion_from_euler(
                math.pi,
                0.0,
                yaw
            )

            aligned_pick_rot = list(aligned_pick_quat)

            # STAP 1: Home & Open Grijper
            feedback_msg.current_state = "initializing"
            goal_handle.publish_feedback(feedback_msg)

            if not self.move_to_state("home", tolerance=0.08, timeout=10.0):
                goal_handle.abort()
                result.success = False
                result.message = "Kon niet naar HOME bewegen."
                return result

            self.open_gripper()

            # STAP 2: Naar 'right'
            feedback_msg.current_state = "moving_to_right"
            goal_handle.publish_feedback(feedback_msg)

            if not self.move_to_state("right", tolerance=0.08, timeout=10.0):
                goal_handle.abort()
                result.success = False
                result.message = "Kon niet naar RIGHT bewegen."
                return result

            # STAP 3A: Naar Pre-Pick via MoveIt
            feedback_msg.current_state = "moving_to_pre_pick"
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info("Beweeg naar Pre-Pick...")

            if not self.move_to_pose_moveit(pre_pick_pos, aligned_pick_rot):
                self.get_logger().error("Pre-pick beweging afgebroken.")
                goal_handle.abort()
                result.success = False
                result.message = "Pre-pick beweging mislukt."
                return result

            # STAP 3B: Zakken naar pick-hoogte via IK
            feedback_msg.current_state = "moving_to_object"
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(
                f"Beweeg nauwkeurig naar pick-hoogte {final_pick_z} m via forced IK..."
            )

            if not self.move_to_pose_forced_ik(
                obj_pos,
                aligned_pick_rot,
                tolerance=0.02,
                timeout=6.0
            ):
                self.get_logger().error("Zakken afgebroken of doel niet nauwkeurig bereikt.")
                goal_handle.abort()
                result.success = False
                result.message = "Zakken naar object mislukt."
                return result

            time.sleep(0.5)

            # STAP 4: Grijpen
            feedback_msg.current_state = "picking_object"
            goal_handle.publish_feedback(feedback_msg)

            self.close_gripper()
            time.sleep(0.5)

            # STAP 5: Lift object omhoog terug naar Pre-Pick
            self.get_logger().info("Lift object omhoog...")

            if not self.move_to_pose_moveit(pre_pick_pos, aligned_pick_rot):
                self.get_logger().error("Liften mislukt. Actie afgebroken.")
                goal_handle.abort()
                result.success = False
                result.message = "Liften mislukt."
                return result

            # STAP 6: Terug naar 'right'
            self.get_logger().info("Beweeg naar veilige tussenpositie ('right')...")

            if not self.move_to_state("right", tolerance=0.08, timeout=10.0):
                goal_handle.abort()
                result.success = False
                result.message = "Kon niet terug naar RIGHT."
                return result

            # STAP 7: Naar sorteerbakje via handmatige IK
            feedback_msg.current_state = "moving_to_bin"
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(f"Start transitie naar bakje ({obj_type})...")

            bakje_xyz = self.bakjes_targets[obj_type]["position"]
            bakje_rpy = self.bakjes_targets[obj_type]["orientation"]

            bakje_quat = tf_transformations.quaternion_from_euler(
                bakje_rpy[0],
                bakje_rpy[1],
                bakje_rpy[2]
            )

            if not self.move_to_pose_forced_ik(
                bakje_xyz,
                list(bakje_quat),
                tolerance=0.08,
                timeout=12.0
            ):
                goal_handle.abort()
                result.success = False
                result.message = f"Kon niet naar bakje voor {obj_type}."
                return result

            # STAP 8: Drop & Terug naar Home
            feedback_msg.current_state = "dropping_object"
            goal_handle.publish_feedback(feedback_msg)

            self.open_gripper()

            if not self.move_to_state("home", tolerance=0.08, timeout=10.0):
                goal_handle.abort()
                result.success = False
                result.message = "Object gedropt, maar terug naar HOME mislukt."
                return result

            goal_handle.succeed()

            result.success = True
            result.message = f"Object van type '{obj_type}' succesvol veilig gesorteerd!"
            return result

        except Exception as e:
            self.get_logger().error(f"❌ Fout tijdens manipulator_task: {e}")

            if goal_handle.is_active:
                goal_handle.abort()

            result.success = False
            result.message = f"Fout tijdens manipulator_task: {e}"
            return result

        finally:
            self.release_robot()

    # =================================================================
    # GO HOME ACTION
    # =================================================================

    def go_home_goal_callback(self, goal_request):
        if not self.acquire_robot("go_home"):
            return GoalResponse.REJECT

        self.get_logger().info("✅ GoHome goal geaccepteerd door Robot.py.")
        return GoalResponse.ACCEPT

    def go_home_cancel_callback(self, goal_handle):
        self.get_logger().warn(
            "🛑 Cancel ontvangen voor go_home. "
            "Huidige home-beweging wordt niet hard onderbroken."
        )
        return CancelResponse.ACCEPT

    def execute_go_home_callback(self, goal_handle):
        self.get_logger().info("🏠 Robot.py voert GoHome uit...")

        feedback_msg = GoHome.Feedback()
        result = GoHome.Result()

        try:
            feedback_msg.current_status = "GoHome gestart in Robot.py..."
            goal_handle.publish_feedback(feedback_msg)

            if not self.wait_for_joint_states(timeout=5.0):
                goal_handle.abort()
                result.success = False
                result.message = "Geen joint states ontvangen."
                return result

            feedback_msg.current_status = "Grijper openen..."
            goal_handle.publish_feedback(feedback_msg)

            self.open_gripper()

            feedback_msg.current_status = "Robot beweegt naar HOME..."
            goal_handle.publish_feedback(feedback_msg)

            home_success = self.move_to_state(
                "home",
                tolerance=0.08,
                timeout=12.0
            )

            if not home_success:
                self.get_logger().error("❌ GoHome mislukt in Robot.py.")

                if goal_handle.is_active:
                    goal_handle.abort()

                result.success = False
                result.message = "GoHome mislukt. Controleer MoveIt/joint limits."
                return result

            feedback_msg.current_status = "Robot staat in HOME."
            goal_handle.publish_feedback(feedback_msg)

            if goal_handle.is_active:
                goal_handle.succeed()

            result.success = True
            result.message = "Robot staat succesvol in HOME."

            self.get_logger().info("✅ GoHome succesvol afgerond in Robot.py.")
            return result

        except Exception as e:
            self.get_logger().error(f"❌ Fout tijdens GoHome in Robot.py: {e}")

            if goal_handle.is_active:
                goal_handle.abort()

            result.success = False
            result.message = f"Fout tijdens GoHome: {e}"
            return result

        finally:
            self.release_robot()

    # =================================================================
    # STARTUP LOOP
    # =================================================================

    def execute_app(self):
        while rclpy.ok() and not self.joint_states_received:
            self.get_logger().info("Synchroniseren met robotposities (/joint_states)...")
            time.sleep(0.5)

        self.get_logger().info("Wachten op binnenkomende actiedoelen...")


def main(args=None):
    rclpy.init(args=args)

    node = manipulatorController("robot_manipulator")

    executor = MultiThreadedExecutor(num_threads=4)
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