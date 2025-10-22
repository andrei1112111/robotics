import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf_transformations
import tf2_ros
import math

class CarrotBroadcaster(Node):
    def __init__(self):
        super().__init__('carrot_broadcaster')
        self.declare_parameter('radius', 1.0)
        self.declare_parameter('direction_of_rotation', 1)

        self.radius = float(self.get_parameter('radius').get_parameter_value().double_value)
        self.direction = int(self.get_parameter('direction_of_rotation').get_parameter_value().integer_value)

        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.angle = 0.0

    def timer_callback(self):
        self.angle += 0.05 * self.direction  # скорость вращения
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'turtle1'
        t.child_frame_id = 'carrot'
        t.transform.translation.x = self.radius * math.cos(self.angle)
        t.transform.translation.y = self.radius * math.sin(self.angle)
        t.transform.translation.z = 0.0
        q = tf_transformations.quaternion_from_euler(0, 0, self.angle)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = CarrotBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
