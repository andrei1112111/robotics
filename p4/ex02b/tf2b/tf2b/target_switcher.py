import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import math

def quaternion_from_yaw(yaw):
    """Преобразование угла yaw в кватернион."""
    qx = 0.0
    qy = 0.0
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qx, qy, qz, qw


class TargetSwitcher(Node):
    def __init__(self):
        super().__init__('target_switcher')

        self.declare_parameter('switch_threshold', 1.0)
        self.switch_threshold = self.get_parameter('switch_threshold').value

        self.br = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)

        self.angle = 0.0

        # Публикация статичной цели
        static_t = TransformStamped()
        static_t.header.stamp = self.get_clock().now().to_msg()
        static_t.header.frame_id = 'world'
        static_t.child_frame_id = 'static_target'
        static_t.transform.translation.x = 8.0
        static_t.transform.translation.y = 2.0
        static_t.transform.translation.z = 0.0
        static_t.transform.rotation.x = 0.0
        static_t.transform.rotation.y = 0.0
        static_t.transform.rotation.z = 0.0
        static_t.transform.rotation.w = 1.0
        self.static_br.sendTransform(static_t)

        self.timer = self.create_timer(0.05, self.timer_callback)


    def timer_callback(self):
        now = self.get_clock().now().to_msg()

        # carrot1 — позади turtle1
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = 'turtle1'
        t.child_frame_id = 'carrot1'
        t.transform.translation.x = 1.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(0.0)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.br.sendTransform(t)

        # carrot2 — вращается вокруг turtle3
        t2 = TransformStamped()
        t2.header.stamp = now
        t2.header.frame_id = 'turtle3'
        t2.child_frame_id = 'carrot2'
        t2.transform.translation.x = 1.5 * math.cos(self.angle)
        t2.transform.translation.y = 1.5 * math.sin(self.angle)
        t2.transform.translation.z = 0.0
        qx, qy, qz, qw = quaternion_from_yaw(self.angle)
        t2.transform.rotation.x = qx
        t2.transform.rotation.y = qy
        t2.transform.rotation.z = qz
        t2.transform.rotation.w = qw
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
