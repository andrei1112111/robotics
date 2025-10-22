import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf_transformations
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import math

class TargetSwitcher(Node):
    def __init__(self):
        super().__init__('target_switcher')
        self.declare_parameter('switch_threshold', 1.0)
        self.switch_threshold = self.get_parameter('switch_threshold').value

        self.br = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)

        # статичная цель
        static_tf = TransformStamped()
        static_tf.header.frame_id = 'world'
        static_tf.child_frame_id = 'static_target'
        static_tf.transform.translation.x = 8.0
        static_tf.transform.translation.y = 2.0
        static_tf.transform.translation.z = 0.0
        static_tf.transform.rotation.w = 1.0
        self.static_br.sendTransform(static_tf)

        self.timer = self.create_timer(0.1, self.publish_targets)
        self.angle = 0.0
        self.get_logger().info(f'Started target_switcher with switch_threshold={self.switch_threshold}')

    def publish_targets(self):
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = 'turtle1'
        t1.child_frame_id = 'carrot1'
        t1.transform.translation.x = math.cos(self.angle)
        t1.transform.translation.y = math.sin(self.angle)
        t1.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, 0)
        t1.transform.rotation.x, t1.transform.rotation.y, t1.transform.rotation.z, t1.transform.rotation.w = q
        self.br.sendTransform(t1)

        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'turtle3'
        t2.child_frame_id = 'carrot2'
        t2.transform.translation.x = math.cos(-self.angle)
        t2.transform.translation.y = math.sin(-self.angle)
        t2.transform.translation.z = 0.0
        t2.transform.rotation.w = 1.0
        self.br.sendTransform(t2)

        self.angle += 0.05

def main(args=None):
    rclpy.init(args=args)
    node = TargetSwitcher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
