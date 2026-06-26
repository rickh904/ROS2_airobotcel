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
from moveit_msgs.msg import MoveItErrorCodes

from interfaces.action import ManipulatorTask, GoHome

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

        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_cb,
            10
        )

        # MoveIt helpers
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

        # Service Clients
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Wachten op MoveIt IK service...')

        self.gripper_client = self.create_client(
            VacuumGripperCtrl,
            '/xarm/set_vacuum_gripper'
        )

        # Sorteerbakjes doelen
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

        # ManipulatorTask action server
        self._action_server = ActionServer(
            self,
            ManipulatorTask,
            'manipulator_task',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # GoHome action server voor HMI home-knop
        self._home_action_server = ActionServer(
            self,
            GoHome,
            '/robot/go_home_action',
            execute_callback=self.execute_go_home_callback,
            goal_callback=self.go_home_goal_callback,
            cancel_callback=self.go_home_cancel_callback
        )

        self.get_logger().info("Lite6 controller succesvol opgestart.")
        self.get_logger().info("GoHome action server actief op /robot/go_home_action.")

    # =================================================================
    # JOINT STATE
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

    def wait_for_motion_done(self, target_joints, tolerance=0.08, timeout=12.0):
        start_time = time.time()

        self.get_logger().info(
            f"Wachten tot beweging klaar is (Tolerantie: {tolerance})..."
        )

        time.sleep(0.4)

        while rclpy.ok():
            if time.time() - start_time > timeout:
                self.get_logger().warn("Timeout bereikt tijdens het wachten op beweging.")
                break

            if self.joint_states_received and self.current_joint_positions:
                error = sum(
                    abs(t - c)
                    for t, c in zip(target_joints, self.current_joint_positions)
                )

                if error < tolerance:
                    self.get_logger().info("Doelpositie fysiek bereikt!")
                    break

            time.sleep(0.1)

    # =================================================================
    # ACTION CALLBACKS
    # =================================================================

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def go_home_goal_callback(self, goal_request):
        self.get_logger().info("GoHome goal ontvangen en geaccepteerd.")
        return GoalResponse.ACCEPT

    def go_home_cancel_callback(self, goal_handle):
        self.get_logger().warn("GoHome cancel ontvangen.")
        return CancelResponse.ACCEPT

    def execute_go_home_callback(self, goal_handle):
        self.get_logger().info("HOME opdracht wordt uitgevoerd...")

        feedback_msg = GoHome.Feedback()
        result = GoHome.Result()

        try:
            if hasattr(feedback_msg, "current_status"):
                feedback_msg.current_status = "Robot beweegt naar HOME..."
                goal_handle.publish_feedback(feedback_msg)

            self.move_to_state(
                "home",
                tolerance=0.08,
                timeout=10.0
            )

            if goal_handle.is_cancel_requested:
                self.get_logger().warn("GoHome is gecanceld.")
                goal_handle.canceled()

                if hasattr(result, "success"):
                    result.success = False

                if hasattr(result, "message"):
                    result.message = "GoHome geannuleerd."

                return result

            goal_handle.succeed()

            if hasattr(result, "success"):
                result.success = True

            if hasattr(result, "message"):
                result.message = "Robot staat in HOME."

            self.get_logger().info("✅ Robot staat in HOME.")

            return result

        except Exception as e:
            self.get_logger().error(f"GoHome mislukt: {e}")

            if goal_handle.is_active:
                goal_handle.abort()

            if hasattr(result, "success"):
                result.success = False

            if hasattr(result, "message"):
                result.message = f"GoHome mislukt: {e}"

            return result

    # =================================================================
    # GRIPPER
    # =================================================================

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

    # =================================================================
    # MOVEIT / IK
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
                self.get_logger().error(f"move_to_pose via MoveIt mislukt: {e}")

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
                self.wait_for_motion_done(
                    target_joints,
                    tolerance=tolerance,
                    timeout=timeout
                )
                return True

            except Exception as e:
                self.get_logger().error(f"Gewrichtsaansturing afgebroken: {e}")
                return False

        self.get_logger().error("Geen geldige IK-oplossing gevonden.")
        return False

    def move_to_state(self, state_name: str, tolerance=0.08, timeout=8.0):
        result, joint_values = self.group_states.get_joint_values(state_name)

        if not result:
            self.get_logger().error(f"State '{state_name}' niet gevonden in SRDF.")
            return False

        self.get_logger().info(f"Moving to state '{state_name}'.")

        self.move_group.move_to_configuration(joint_values)
        self.wait_for_motion_done(
            joint_values,
            tolerance=tolerance,
            timeout=timeout
        )

        return True

    # =================================================================
    # ROBUUSTE ROTATIEBEREKENING
    # =================================================================

    def calculate_robust_pick_rotation(self, goal_rotation):
        """
        Deze functie rekent de grijperhoek robuust uit.

        Belangrijk:
        De transformatie-node stuurt de yaw NIET als normale quaternion.
        Hij stuurt:
            rotation.x = cos(yaw / 2)
            rotation.y = sin(yaw / 2)
            rotation.z = 0
            rotation.w = 0

        Daarom gebruiken we GEEN euler_from_quaternion().
        We rekenen yaw direct terug uit x/y:
            yaw = 2 * atan2(y, x)
        """

        encoded_x = float(goal_rotation.x)
        encoded_y = float(goal_rotation.y)

        magnitude = math.sqrt(encoded_x ** 2 + encoded_y ** 2)

        if magnitude < 0.001:
            self.get_logger().warn(
                "⚠️ Ongeldige yaw-code ontvangen. Fallback naar yaw=0 graden."
            )
            yaw = 0.0

        else:
            encoded_x = encoded_x / magnitude
            encoded_y = encoded_y / magnitude

            yaw = 2.0 * math.atan2(encoded_y, encoded_x)

        # Normaliseren naar -180 tot +180 graden
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))

        # Extra robuust afronden op stappen van 15 graden
        yaw_step = math.radians(15.0)
        yaw = round(yaw / yaw_step) * yaw_step

        # Grijper is symmetrisch over 180 graden.
        # Daarom houden we de hoek binnen -90 tot +90 graden.
        yaw_deg = math.degrees(yaw)

        while yaw_deg > 90.0:
            yaw_deg -= 180.0

        while yaw_deg <= -90.0:
            yaw_deg += 180.0

        yaw = math.radians(yaw_deg)

        self.get_logger().info(
            f"✅ Robuuste pick-yaw berekend: "
            f"{yaw_deg:.1f} graden "
            f"(input rotation.x={goal_rotation.x:.3f}, "
            f"rotation.y={goal_rotation.y:.3f}, "
            f"rotation.z={goal_rotation.z:.3f}, "
            f"rotation.w={goal_rotation.w:.3f})"
        )

        # Gripper recht naar beneden + berekende yaw
        aligned_pick_quat = tf_transformations.quaternion_from_euler(
            math.pi,
            0.0,
            yaw
        )

        return list(aligned_pick_quat)

    # =================================================================
    # STARTUP
    # =================================================================

    def execute_app(self):
        while rclpy.ok() and not self.joint_states_received:
            self.get_logger().info("Synchroniseren met robotposities (/joint_states)...")
            time.sleep(0.5)

        self.get_logger().info("Wachten op binnenkomende actiedoelen...")

    # =================================================================
    # MANIPULATOR TASK
    # =================================================================

    def execute_callback(self, goal_handle):
        self.get_logger().info('Opdracht wordt nu uitgevoerd...')

        feedback_msg = ManipulatorTask.Feedback()
        result = ManipulatorTask.Result()
        goal = goal_handle.request

        # HARDCODED HOOGTE: 80 mm boven de tafel
        final_pick_z = 0.08

        pre_pick_pos = [
            goal.position.x,
            goal.position.y,
            0.15
        ]

        obj_pos = [
            goal.position.x,
            goal.position.y,
            final_pick_z
        ]

        obj_type = goal.object_type

        if obj_type not in self.bakjes_targets:
            self.get_logger().error(f"Onbekend object type: {obj_type}")
            goal_handle.abort()
            result.success = False
            result.message = f"Onbekend object type: {obj_type}"
            return result

        # ROTATIE ROBUUST BEREKENEN
        # Oude euler_from_quaternion() eruit gehaald.
        aligned_pick_rot = self.calculate_robust_pick_rotation(goal.rotation)

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
            result.message = "Pre-pick beweging mislukt."
            return result

        # STAP 3B: Gecontroleerd en nauwkeurig zakken naar de harde pick-hoogte via Handmatige IK
        feedback_msg.current_state = "moving_to_object"
        goal_handle.publish_feedback(feedback_msg)

        self.get_logger().info(
            f"Beweeg nauwkeurig naar harde pick-hoogte van {final_pick_z} m via forced IK..."
        )

        if not self.move_to_pose_forced_ik(
            obj_pos,
            aligned_pick_rot,
            tolerance=0.02,
            timeout=6.0
        ):
            self.get_logger().error("Zakken afgebroken of doel niet nauwkeurig bereikt!")

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

        # STAP 5: Lift object recht omhoog terug naar Pre-Pick
        self.get_logger().info("Lift object omhoog...")

        if not self.move_to_pose_moveit(pre_pick_pos, aligned_pick_rot):
            self.get_logger().error("Liften mislukt. Actie afgebroken.")

            goal_handle.abort()
            result.success = False
            result.message = "Liften mislukt."
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

        bakje_quat = tf_transformations.quaternion_from_euler(
            bakje_rpy[0],
            bakje_rpy[1],
            bakje_rpy[2]
        )

        self.move_to_pose_forced_ik(
            bakje_xyz,
            list(bakje_quat),
            tolerance=0.08,
            timeout=12.0
        )

        # STAP 8: Drop & Terug naar Home
        feedback_msg.current_state = "dropping_object"
        goal_handle.publish_feedback(feedback_msg)

        self.open_gripper()

        self.move_to_state(
            "home",
            tolerance=0.08,
            timeout=10.0
        )

        goal_handle.succeed()

        result.success = True
        result.message = f"Object van type '{obj_type}' succesvol veilig gesorteerd!"

        return result


def main(args=None):
    rclpy.init(args=args)

    node = manipulatorController("demo")

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    executor_thread = Thread(
        target=executor.spin,
        daemon=True
    )

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