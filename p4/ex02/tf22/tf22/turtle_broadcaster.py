import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import TransformStamped
import tf_transformations
import tf2_ros

class TurtleBroadcaster(Node):
    def __init__(self):
        super().__init__('turtle_broadcaster')
        self.declare_parameter('turtle', 'turtle1')
        self.turtle_name = self.get_parameter('turtle').get_parameter_value().string_value

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.subscription = self.create_subscription(Pose, f'/{self.turtle_name}/pose', self.handle_turtle_pose, 10)

    def handle_turtle_pose(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = self.turtle_name
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = TurtleBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
