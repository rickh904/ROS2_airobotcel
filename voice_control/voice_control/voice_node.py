import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from airobot_interfaces.action import AutoSort
from airobot_interfaces.action import SortSpec


import serial
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
            'autosort'
        )

        self.sort_spec_client = ActionClient(
            self,
            SortSpec,
            'sort_spec'
        )
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
        if command in ['start', 'stop', 'reset']:
            self.send_auto_sort_goal(command)
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

    def send_auto_sort_goal(self, command):
        if not self.auto_sort_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Action server auto_sort niet beschikbaar')
            return

        goal_msg = AutoSort.Goal()
        goal_msg.command = command

        self.get_logger().info(f'Verstuur auto_sort action goal: {command}')
        self.auto_sort_client.send_goal_async(goal_msg)

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
