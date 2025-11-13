#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleMovement(Node):
    def __init__(self):
        super().__init__('circle_movement')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('CircleMovement started')
        self.time = 0.0

    def timer_callback(self):
        msg = Twist()
        
        t = self.time
        msg.linear.x = 0.3
        msg.angular.z = 2 * math.sin(t)
        self.pub.publish(msg)
        self.time += 0.1

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
