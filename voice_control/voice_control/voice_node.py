#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String, Bool
from interfaces.action import AutoSort

import serial


class VoiceNode(Node):
    def __init__(self):
        super().__init__('voice_node')

        self.voice_command_pub = self.create_publisher(String, '/voice_command', 10)
        self.stop_pub = self.create_publisher(Bool, '/system/stop_request', 10)

        self.auto_sort_client = ActionClient(self, AutoSort, 'AutoSort')
        self.auto_sort_goal_handle = None

        self.valid_commands = ['start', 'stop', 'reset']

        self.serial_port = None
        self.connect_to_esp32()

        self.timer = self.create_timer(0.1, self.read_serial)

        self.get_logger().info('🎤 Voice node gestart')
        self.get_logger().info('🎤 START -> AutoSort action')
        self.get_logger().info('🎤 STOP  -> /system/stop_request')

    def connect_to_esp32(self):
        possible_ports = [
            '/dev/ttyACM0',
            '/dev/ttyACM1',
            '/dev/ttyUSB0',
            '/dev/ttyUSB1',
        ]

        for port in possible_ports:
            try:
                self.serial_port = serial.Serial(
                    port=port,
                    baudrate=115200,
                    timeout=0.1
                )

                self.get_logger().info(f'✅ Verbonden met ESP32 op {port}')
                return

            except serial.SerialException:
                pass

        self.get_logger().error(
            '❌ Geen ESP32 gevonden op /dev/ttyACM0, /dev/ttyACM1, /dev/ttyUSB0 of /dev/ttyUSB1'
        )

    def read_serial(self):
        if self.serial_port is None:
            return

        try:
            if self.serial_port.in_waiting <= 0:
                return

            raw_line = self.serial_port.readline()

            line = (
                raw_line
                .decode(errors='ignore')
                .strip()
                .lower()
            )

            if line == '':
                return

            self.get_logger().info(f'🎤 Ontvangen van ESP32: {line}')

            command = self.extract_command(line)

            if command is None:
                self.get_logger().warn(f'⚠️ Geen geldig voice command gevonden in: {line}')
                return

            self.publish_voice_command(command)
            self.handle_command(command)

        except serial.SerialException as error:
            self.get_logger().error(f'❌ Serial fout: {error}')
            self.serial_port = None

    def extract_command(self, line):
        if 'robotcommando:' in line:
            command_text = line.split('robotcommando:', 1)[1].strip()
        else:
            command_text = line.strip()

        for command in self.valid_commands:
            if command_text == command:
                return command

        return None

    def publish_voice_command(self, command):
        msg = String()
        msg.data = command
        self.voice_command_pub.publish(msg)

        self.get_logger().info(f'📢 Gepubliceerd op /voice_command: {command}')

    def handle_command(self, command):
        if command == 'start':
            self.send_auto_sort_start_goal()
            return

        if command == 'stop':
            self.send_stop_request()
            self.cancel_own_auto_sort_goal()
            return

        if command == 'reset':
            self.get_logger().warn(
                'Reset ontvangen via voice, maar reset is nog niet gekoppeld.'
            )
            return

    def send_auto_sort_start_goal(self):
        if not self.auto_sort_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('❌ AutoSort action server niet beschikbaar')
            return

        goal_msg = AutoSort.Goal()
        goal_msg.start_request = True

        self.get_logger().info('▶️ Voice START: AutoSort goal versturen')

        send_goal_future = self.auto_sort_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.auto_sort_goal_response_callback)

    def auto_sort_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('⚠️ AutoSort goal geweigerd door main_controller')
            self.auto_sort_goal_handle = None
            return

        self.auto_sort_goal_handle = goal_handle
        self.get_logger().info('✅ AutoSort goal geaccepteerd door main_controller')

    def send_stop_request(self):
        msg = Bool()
        msg.data = True

        self.stop_pub.publish(msg)

        self.get_logger().warn('🛑 Voice STOP: True gepubliceerd op /system/stop_request')

    def cancel_own_auto_sort_goal(self):
        if self.auto_sort_goal_handle is None:
            self.get_logger().warn(
                '🛑 Geen opgeslagen AutoSort goal om te cancelen. STOP-topic is wel verzonden.'
            )
            return

        self.get_logger().warn('🛑 Voice STOP: actieve AutoSort goal annuleren')

        cancel_future = self.auto_sort_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        cancel_response = future.result()

        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('✅ AutoSort goal succesvol geannuleerd')
        else:
            self.get_logger().warn(
                '⚠️ AutoSort goal kon niet worden geannuleerd of was al klaar'
            )

        self.auto_sort_goal_handle = None


def main(args=None):
    rclpy.init(args=args)

    node = VoiceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial_port is not None:
            node.serial_port.close()

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
