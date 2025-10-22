import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf_transformations
import tf2_ros
from turtlesim.msg import Pose


class FramePublisher(Node):
    def __init__(self):
        super().__init__('turtle1_broadcaster')
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.handle_turtle_pose,
            10)
        self.subscription

    def handle_turtle_pose(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'turtle1'
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FramePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()