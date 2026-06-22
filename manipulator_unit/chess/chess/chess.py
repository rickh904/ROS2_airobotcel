#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess
from std_msgs.msg import String

from my_moveit_python import srdfGroupStates
from my_moveit_python import MovegroupHelper

from xarm_msgs.srv import VacuumGripperCtrl

from threading import Thread

prefix = ''
joint_names = [
        prefix + "joint1",
        prefix + "joint2",
        prefix + "joint3",
        prefix + "joint4",
        prefix + "joint5",
        prefix + "joint6",
    ]
base_link_name = "link_base"
end_effector_name = "link6"
group_name = "lite6"
package_name = 'my_uf_moveit_config'
srdf_file_name = 'config/uf_robot.srdf'


board_center = [0.285, 0.0]

field_size = 0.03
half_field_size = field_size / 2
half_board_size = (field_size * 8) / 2

board_a1 =[0, 0]
board_a1[0] = board_center[0] + half_board_size + field_size
board_a1[1] = board_center[1] - half_board_size + field_size


pre_grasp_height = 0.2
drop_height = 0.1

class VacuumGripperClient(Node):
    def __init__(self):
        super().__init__('vacuum_gripper_client')
        self.client = self.create_client(VacuumGripperCtrl, '/xarm/set_vacuum_gripper')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.get_logger().info('Service is available.')

    def send_request(self, on_state):
        request = VacuumGripperCtrl.Request()
        request.on = on_state
        self.future = self.client.call_async(request)
        return self.future
    
    def open(self):
        self.send_request(True)
    def close(self):
        self.send_request(False)

class ChessNode(Node):
    def __init__(self, node):
        self.node = node
        super().__init__('chess_node')
        self.get_logger().info('Chess node started')

        self.gripper = VacuumGripperClient();

        # ROS 2 publisher for the best move
        self.publisher_ = self.create_publisher(
            String, 'chess_best_move', 10
        )  # Publish to the topic `chess_best_move`

        # Timer to control the loop
        self.timer = self.create_timer(10.0, self.timer_callback)

        # Start Stockfish subprocess once and reuse
        self.stockfish_process = self.start_stockfish_process()
        self.current_position = 'startpos'
        self.move_group_helper = MovegroupHelper(self.node, joint_names, base_link_name, end_effector_name, group_name)
        self.uf_groupstates = srdfGroupStates(package_name, srdf_file_name, group_name)

        # Spin the node in background thread(s) and wait a bit for initialization
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(node)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        node.create_rate(1.0).sleep()

        
        result, joint_values = self.uf_groupstates.get_joint_values('home')
        if result:
            print("Move to home")
            self.move_group_helper.move_to_configuration(joint_values)
        else:
            print( "Failed to get joint_values of home")
        print("Open gripper")
        self.gripper.open();

    def start_stockfish_process(self):
        """Start Stockfish as a persistent subprocess."""
        process = subprocess.Popen(
            ['stockfish'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
        )
        # Initialize Stockfish with UCI protocol
        process.stdin.write('uci\n')
        process.stdin.flush()

        # Wait for 'uciok'
        while True:
            output = process.stdout.readline().strip()
            if output == 'uciok':
                break

        return process

    def timer_callback(self):
        """ROS 2 timer callback to compute and publish the best move."""
        best_move = self.get_best_move(self.current_position)
        translation_start = [0.5, 0.1, 0.1]
        rotation_start = [1.0, 0.0, 0.0, 0.0]
        translation_end = [0.5, 0.1, 0.1]
        rotation_end = [1.0, 0.0, 0.0, 0.0]
        if best_move:
            start_position = best_move[:2]
            end_position = best_move[2:]
            self.get_logger().info(f'Best move: From {start_position} To {end_position}')
            self.current_position += f' {best_move}'

            translation_start[0] = board_a1[0] - (int(start_position[1]) * field_size)
            translation_start[1] = board_a1[1] + ((ord(start_position[0]) - 97) * field_size)
            translation_start[2] = pre_grasp_height
            self.move_group_helper.move_to_pose(translation_start, rotation_start)
            translation_start[2] = drop_height
            self.move_group_helper.move_to_pose(translation_start, rotation_start)
            self.gripper.close()
            translation_start[2] = pre_grasp_height
            self.move_group_helper.move_to_pose(translation_start, rotation_start)

            translation_end[0] = board_a1[0] - (int(end_position[1]) * field_size)
            translation_end[1] = board_a1[1] + ((ord(end_position[0]) - 97) * field_size)
            translation_end[2] = pre_grasp_height
            self.move_group_helper.move_to_pose(translation_end, rotation_end)
            translation_end[2] = drop_height
            self.move_group_helper.move_to_pose(translation_end, rotation_end)
            print("Open gripper")
            self.gripper.open()
            translation_end[2] = pre_grasp_height
            self.move_group_helper.move_to_pose(translation_end, rotation_end)


            msg = String();
            msg.data = f'{start_position}->{end_position}'

            # Publish the move
            self.publisher_.publish(msg)

    def get_best_move(self, current_position):
        """Get the best move from Stockfish for the given position."""
        # Set the current position in Stockfish
        self.stockfish_process.stdin.write(f'position {current_position}\n')
        self.stockfish_process.stdin.write('go movetime 1000\n')  # Search for 1 second
        self.stockfish_process.stdin.flush()

        # Read Stockfish output and find the best move
        best_move = None
        while True:
            output = self.stockfish_process.stdout.readline().strip()
            if output.startswith('bestmove'):
                best_move = output.split(' ')[1]
                break

        return best_move

    def destroy_node(self):
        """Clean up resources on shutdown."""
        # Quit Stockfish gracefully
        if self.stockfish_process:
            self.stockfish_process.stdin.write('quit\n')
            self.stockfish_process.stdin.flush()
            self.stockfish_process.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Node("demo")
    chess_node = ChessNode(node)
    try:
        rclpy.spin(chess_node)
    except KeyboardInterrupt:
        pass
    finally:
        chess_node.destroy_node()
        rclpy.shutdown()
