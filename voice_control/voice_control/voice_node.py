import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from interfaces.action import AutoSort
from airobot_interfaces.action import SortSpec

import serial


START_TEST_DELAY_SEC = 10.0


class VoiceNode(Node):

    def __init__(self):
        super().__init__('voice_node')

        self.publisher_ = self.create_publisher(
            String,
            '/voice_command',
            10
        )

        self.auto_sort_client = ActionClient(
            self,
            AutoSort,
            'AutoSort'
        )

        self.sort_spec_client = ActionClient(
            self,
            SortSpec,
            'sort_spec'
        )

        # Hier bewaren we de actieve AutoSort-goal.
        # Deze is nodig zodat het voice-commando "stop" de actieve goal kan annuleren.
        self.auto_sort_goal_handle = None

        # TEST:
        # Na voice-commando "start" wachten we 10 seconden voordat AutoSort echt start.
        # In die 10 seconden kun jij "stop" zeggen om te testen of stop binnenkomt.
        self.start_delay_timer = None
        self.start_delay_active = False

        self.valid_commands = [
            'start',
            'stop',
            'reset',
            'pick_oral_b_head',
            'pick_aaa_battery',
            'pick_m6_bolt',
            'pick_wall_plug',
        ]

        self.serial_port = None

        self.connect_to_esp32()

        self.timer = self.create_timer(0.1, self.read_serial)

        self.get_logger().info('Voice node gestart')
        self.get_logger().info('Publiceert geldige commandos op /voice_command')
        self.get_logger().warn(
            f'TESTMODUS actief: na start wacht voice {START_TEST_DELAY_SEC:.0f} seconden. '
            'Zeg in die tijd stop om te testen of stop binnenkomt.'
        )

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

                self.get_logger().info(f'Verbonden met ESP32 op {port}')
                return

            except serial.SerialException:
                pass

        self.get_logger().error(
            'Geen ESP32 gevonden. Sluit de ESP32 aan en verbind USB met Ubuntu VM.'
        )

    def read_serial(self):
        if self.serial_port is None:
            return

        try:
            if self.serial_port.in_waiting > 0:
                raw_line = self.serial_port.readline()

                line = (
                    raw_line
                    .decode(errors='ignore')
                    .strip()
                    .lower()
                )

                if line == '':
                    return

                self.get_logger().info(f'Ontvangen van ESP32: {line}')
                command = self.extract_command(line)

                if command is None:
                    self.get_logger().warn(f'Geen geldig voice command gevonden in: {line}')
                    return

                msg = String()
                msg.data = command
                self.publisher_.publish(msg)

                self.send_action(command)

                self.get_logger().info(f'Gepubliceerd op /voice_command: {command}')

        except serial.SerialException as error:
            self.get_logger().error(f'Serial fout: {error}')
            self.serial_port = None

    def extract_command(self, line):
        if 'robotcommando:' not in line:
            return None

        command_text = line.split('robotcommando:', 1)[1].strip()

        for command in self.valid_commands:
            if command_text == command:
                return command

        return None

    def send_action(self, command):
        if command == 'start':
            self.start_delay_before_auto_sort()
            return

        if command == 'stop':
            self.handle_stop_command()
            return

        if command == 'reset':
            self.get_logger().warn(
                'Reset ontvangen via voice. Reset wordt voorlopig alleen gepubliceerd op /voice_command.'
            )
            return

        product_map = {
            'pick_oral_b_head': 'oral_b_head',
            'pick_aaa_battery': 'aaa_battery',
            'pick_m6_bolt': 'm6_bolt',
            'pick_wall_plug': 'wall_plug',
        }

        if command in product_map:
            self.send_sort_spec_goal(product_map[command])
            return

        self.get_logger().warn(f'Geen action gekoppeld aan command: {command}')

    def start_delay_before_auto_sort(self):
        if self.start_delay_active:
            self.get_logger().warn(
                'Start ontvangen, maar er loopt al een 10 seconden start-testvenster.'
            )
            return

        self.start_delay_active = True

        self.get_logger().warn('==============================================')
        self.get_logger().warn(
            f'START ontvangen. AutoSort start nog NIET direct.'
        )
        self.get_logger().warn(
            f'Je hebt nu {START_TEST_DELAY_SEC:.0f} seconden om STOP te zeggen.'
        )
        self.get_logger().warn(
            'Als STOP binnenkomt, zie je dat hier in de voice terminal.'
        )
        self.get_logger().warn(
            f'Als er geen STOP komt, start AutoSort na {START_TEST_DELAY_SEC:.0f} seconden alsnog.'
        )
        self.get_logger().warn('==============================================')

        self.start_delay_timer = self.create_timer(
            START_TEST_DELAY_SEC,
            self.start_delay_done_callback
        )

    def start_delay_done_callback(self):
        self.clear_start_delay_timer()

        self.get_logger().warn(
            f'{START_TEST_DELAY_SEC:.0f} seconden voorbij zonder stop. '
            'AutoSort wordt nu normaal gestart.'
        )

        self.send_auto_sort_start_goal()

    def clear_start_delay_timer(self):
        if self.start_delay_timer is not None:
            self.start_delay_timer.cancel()
            self.destroy_timer(self.start_delay_timer)
            self.start_delay_timer = None

        self.start_delay_active = False

    def handle_stop_command(self):
        if self.start_delay_active:
            self.get_logger().warn('==============================================')
            self.get_logger().warn(
                'STOP IS BINNENGEKOMEN TIJDENS HET 10 SECONDEN TESTVENSTER.'
            )
            self.get_logger().warn(
                'Dit betekent dat voice stop goed ontvangt vanaf de ESP32.'
            )
            self.get_logger().warn(
                'AutoSort wordt nu NIET gestart.'
            )
            self.get_logger().warn('==============================================')

            self.clear_start_delay_timer()
            return

        self.cancel_auto_sort_goal()

    def send_auto_sort_start_goal(self):
        if not self.auto_sort_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Action server AutoSort niet beschikbaar')
            return

        goal_msg = AutoSort.Goal()
        goal_msg.start_request = True

        self.get_logger().info('Verstuur AutoSort action goal: start_request=True')

        send_goal_future = self.auto_sort_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.auto_sort_goal_response_callback)

    def auto_sort_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().warn('AutoSort goal geweigerd door main_controller')
            self.auto_sort_goal_handle = None
            return

        self.auto_sort_goal_handle = goal_handle
        self.get_logger().info('AutoSort goal geaccepteerd en opgeslagen voor stop/cancel')

    def cancel_auto_sort_goal(self):
        if self.auto_sort_goal_handle is None:
            self.get_logger().warn(
                'Stop ontvangen, maar er is geen actieve AutoSort goal om te annuleren'
            )
            return

        self.get_logger().warn('Stop ontvangen: actieve AutoSort goal annuleren')

        cancel_future = self.auto_sort_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self.cancel_done_callback)

    def cancel_done_callback(self, future):
        cancel_response = future.result()

        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('AutoSort goal succesvol geannuleerd')
        else:
            self.get_logger().warn(
                'AutoSort goal kon niet worden geannuleerd of was al klaar'
            )

        self.auto_sort_goal_handle = None

    def send_sort_spec_goal(self, product_type):
        if not self.sort_spec_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Action server sort_spec niet beschikbaar')
            return

        goal_msg = SortSpec.Goal()
        goal_msg.product_type = product_type

        self.get_logger().info(f'Verstuur sort_spec action goal: {product_type}')
        self.sort_spec_client.send_goal_async(goal_msg)


def main(args=None):
    rclpy.init(args=args)

    node = VoiceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    if node.serial_port is not None:
        node.serial_port.close()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
