import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class TextToCmdVel(Node):
    def __init__(self):
        super().__init__('text_to_cmd_vel')
        self.subscription = self.create_subscription(
            String,
            'cmd_text',
            self.cmd_text_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

    def cmd_text_callback(self, msg):
        twist = Twist()
        if msg.data == 'move_forward':
            twist.linear.x = 1.0
        elif msg.data == 'move_backward':
            twist.linear.x = -1.0
        elif msg.data == 'turn_left':
            twist.angular.z = 1.5
        elif msg.data == 'turn_right':
            twist.angular.z = -1.5
        else:
            self.get_logger().warn(f'Unknown command: {msg.data}')
        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TextToCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
