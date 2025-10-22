import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import math

class TurtleListener(Node):
    def __init__(self):
        super().__init__('turtle_listener')
        self.publisher = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.1, self.on_timer)

    def on_timer(self):
        try:
            trans = self.tf_buffer.lookup_transform('turtle2', 'carrot', rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        msg = Twist()
        msg.angular.z = 4.0 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
        msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x**2 + trans.transform.translation.y**2)
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = TurtleListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
