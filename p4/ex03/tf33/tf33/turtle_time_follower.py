import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from collections import deque
import math
import time

class TurtleFollower(Node):
    def __init__(self):
        super().__init__('turtle_follower')

        # параметры
        self.declare_parameter('delay', 1.0)
        self.delay = self.get_parameter('delay').value

        self.pose_history = deque()  # (timestamp, x, y, theta)
        self.target_pose = None

        # подписка на позу первой черепахи
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        # публикация скорости для второй
        self.pub = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)

        # текущая поза второй черепахи
        self.turtle2_pose = None
        self.sub2 = self.create_subscription(Pose, '/turtle2/pose', self.pose2_callback, 10)

        # таймер на управление
        self.timer = self.create_timer(0.05, self.control_loop)

    def pose_callback(self, msg):
        """Сохраняем позу первой черепахи с временем."""
        self.pose_history.append((time.time(), msg.x, msg.y, msg.theta))

        # очищаем старые позы
        while self.pose_history and time.time() - self.pose_history[0][0] > self.delay:
            self.target_pose = self.pose_history.popleft()

    def pose2_callback(self, msg):
        self.turtle2_pose = msg

    def control_loop(self):
        if self.target_pose is None or self.turtle2_pose is None:
            return

        _, x_tgt, y_tgt, _ = self.target_pose
        x2 = self.turtle2_pose.x
        y2 = self.turtle2_pose.y
        theta2 = self.turtle2_pose.theta

        dx = x_tgt - x2
        dy = y_tgt - y2
        distance = math.hypot(dx, dy)
        angle_to_target = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_target - theta2)

        msg = Twist()
        if distance > 0.05:
            msg.linear.x = 1.5 * distance
            msg.angular.z = 4.0 * angle_diff
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.pub.publish(msg)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = TurtleFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()