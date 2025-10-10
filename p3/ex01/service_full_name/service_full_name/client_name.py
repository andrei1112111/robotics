#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from fullname_interfaces.srv import SummFullName


class FullNameClient(Node):
    def __init__(self):
        super().__init__('client_name')
        self.client = self.create_client(SummFullName, 'SummFullName')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service "SummFullName"...')

    def send_request(self, surname: str, name: str, patronymic: str):
        req = SummFullName.Request()
        req.surname = surname
        req.name = name
        req.patronymic = patronymic
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    node = FullNameClient()
    if len(sys.argv) < 4:
        node.get_logger().info('Usage: ros2 run service_full_name client_name <surname> <name> <patronymic>')
        node.destroy_node()
        rclpy.shutdown()
        return

    surname = sys.argv[1]
    name = sys.argv[2]
    patronymic = sys.argv[3]
    resp = node.send_request(surname, name, patronymic)
    if resp is not None:
        node.get_logger().info(f"Full name received: '{resp.full_name}'")
    else:
        node.get_logger().error('Service call failed or returned no response')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

