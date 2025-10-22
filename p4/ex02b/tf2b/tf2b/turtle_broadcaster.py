import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from turtlesim.msg import Pose
from rclpy.time import Time
import math

class TurtleBroadcaster(Node):
    def __init__(self):
        super().__init__('turtle_broadcaster')

        self.turtle_name = self.declare_parameter('turtle', 'turtle1').get_parameter_value().string_value
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Pose, f'/{self.turtle_name}/pose', self.handle_turtle_pose, 10)
        self.get_logger().info(f"Started broadcaster for {self.turtle_name}")

    def handle_turtle_pose(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.turtle_name
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(msg.theta / 2.0)
        t.transform.rotation.w = math.cos(msg.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = TurtleBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
