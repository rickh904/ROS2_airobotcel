import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from airobot_interfaces.action import AutoSort
from airobot_interfaces.action import SortSpec


class HoofdController(Node):
    def __init__(self):
        super().__init__('hoofdcontroller')

        self.state = 'idle'

        self.auto_sort_server = ActionServer(
            self,
            AutoSort,
            'auto_sort',
            self.execute_auto_sort
        )

        self.sort_spec_server = ActionServer(
            self,
            SortSpec,
            'sort_spec',
            self.execute_sort_spec
        )

        self.get_logger().info('Hoofdcontroller gestart')
        self.get_logger().info('Action server actief: /auto_sort')
        self.get_logger().info('Action server actief: /sort_spec')

    def execute_auto_sort(self, goal_handle):
        command = goal_handle.request.command.lower()

        self.get_logger().info(f'Auto_sort ontvangen: {command}')

        result = AutoSort.Result()

        if command == 'start':
            self.state = 'running'
            result.success = True
            result.message = 'Start ontvangen'

        elif command == 'stop':
            self.state = 'stopped'
            result.success = True
            result.message = 'Stop ontvangen'

        elif command == 'reset':
            self.state = 'idle'
            result.success = True
            result.message = 'Reset ontvangen'

        else:
            result.success = False
            result.message = f'Onbekend auto_sort commando: {command}'
            goal_handle.abort()
            return result

        goal_handle.succeed()
        self.get_logger().info(result.message)
        self.get_logger().info(f'Huidige state: {self.state}')
        return result

    def execute_sort_spec(self, goal_handle):
        product = goal_handle.request.product_type.lower()

        self.get_logger().info(f'Sort_spec ontvangen: {product}')

        result = SortSpec.Result()

        valid_products = [
            'oral_b_head',
            'aaa_battery',
            'm6_bolt',
            'wall_plug'
        ]

        if product in valid_products:
            self.state = 'sort_specific_product'
            result.success = True
            result.message = f'Product ontvangen: {product}'
            goal_handle.succeed()
        else:
            result.success = False
            result.message = f'Onbekend product: {product}'
            goal_handle.abort()

        self.get_logger().info(result.message)
        self.get_logger().info(f'Huidige state: {self.state}')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = HoofdController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
