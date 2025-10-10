#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from fullname_interfaces.srv import SummFullName


class FullNameService(Node):
    def __init__(self):
        super().__init__('service_name')
        self.srv = self.create_service(SummFullName, 'SummFullName', self.handle_summ_fullname)
        self.get_logger().info('Service "SummFullName" ready')

    def handle_summ_fullname(self, request, response):
        # concat with spaces, strip extra spaces
        full = f"{request.surname} {request.name} {request.patronymic}".strip()
        response.full_name = full
        self.get_logger().info(f"Request -> {request.surname!r}, {request.name!r}, {request.patronymic!r} -> {full!r}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = FullNameService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

