import rclpy

from rclpy.node import Node

from std_msgs.msg import String



import serial
class VoiceNode(Node):

    def __init__(self):

        super().__init__('voice_node')



        self.publisher_ = self.create_publisher(

            String,

            '/voice_command',

            10

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



               self.get_logger().info(f'Gepubliceerd op /voice_command: {command}')



        except serial.SerialException as error:

            self.get_logger().error(f'Serial fout: {error}')

            self.serial_port = None
    def extract_command(self, line):

        for command in self.valid_commands:

            if command in line:

                return command



        return None





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
