import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import atan2, sqrt, pi
import sys


class MoveToGoal(Node):

    def __init__(self, x_goal, y_goal, theta_goal_deg):
        super().__init__("move_to_goal")

        self.goal_x = float(x_goal)
        self.goal_y = float(y_goal)
        self.goal_theta = float(theta_goal_deg) * pi / 180.0

        self.pose = None
        self.cmd_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)

        self.timer = self.create_timer(0.1, self.loop)

        self.get_logger().info(
            f"Goal: x={self.goal_x}, y={self.goal_y}, theta={theta_goal_deg}°")

    def normalize(self, a):
        while a > pi:
            a -= 2*pi
        while a < -pi:
            a += 2*pi
        return a

    def pose_callback(self, msg):
        self.pose = msg

    def loop(self):
        if self.pose is None:
            return

        dx = self.goal_x - self.pose.x
        dy = self.goal_y - self.pose.y
        distance = sqrt(dx*dx + dy*dy)

        angle_to_goal = atan2(dy, dx)
        angle_error = self.normalize(angle_to_goal - self.pose.theta)

        cmd = Twist()

        # rotate toward target 
        if abs(angle_error) > 0.1 and distance > 0.3:
            cmd.linear.x = 0.0
            cmd.angular.z = 2.0 * angle_error
            self.cmd_pub.publish(cmd)
            return

        # move straight to target 
        if distance > 0.3:
            cmd.linear.x = min(1.5 * distance, 2.0)   # strong forward
            cmd.angular.z = 1.5 * angle_error         # minor orientation correction
            self.cmd_pub.publish(cmd)
            return

        # final rotation 
        theta_error = self.normalize(self.goal_theta - self.pose.theta)
        if abs(theta_error) > 0.05:
            cmd.linear.x = 0.0
            cmd.angular.z = 1.5 * theta_error
            self.cmd_pub.publish(cmd)
            return

        # 
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
        self.get_logger().info("GOAL REACHED")
        rclpy.shutdown()


def main():
    rclpy.init()
    if len(sys.argv) != 4:
        print("Usage: ros2 run move_to_goal move_to_goal x y theta_deg")
        return

    node = MoveToGoal(sys.argv[1], sys.argv[2], sys.argv[3])
    rclpy.spin(node)


if __name__ == "__main__":
    main()
