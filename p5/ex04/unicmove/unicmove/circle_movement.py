#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleMovement(Node):
    def __init__(self):
        super().__init__('circle_movement')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('CircleMovement started')

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.2
        msg.angular.z = 0.5
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CircleMovement()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
