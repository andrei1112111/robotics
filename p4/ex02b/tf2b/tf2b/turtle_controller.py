#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import math, sys, select, termios, tty, os

def is_interactive():
    return sys.stdin.isatty() and os.getenv("TERM") is not None

def get_key(timeout=0.1):
    if not is_interactive():
        return None
    dr, dw, de = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return None

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.current_target_pub = self.create_publisher(Pose, '/current_target', 10)

        # Список целей
        self.targets = ['carrot1', 'carrot2', 'static_target']
        self.current_target_index = 0
        self.current_target = self.targets[self.current_target_index]

        self.switch_threshold = 1.0
        self.get_logger().info("TurtleController started.")

        self.timer = self.create_timer(0.1, self.control_loop)

        self.switch_to_next_target()
        self.get_logger().info("~~~ " + str(is_interactive()) + " ~~~")

    def control_loop(self):
        try:
            trans = self.tf_buffer.lookup_transform('turtle2', self.current_target, rclpy.time.Time())

            dx = trans.transform.translation.x
            dy = trans.transform.translation.y
            distance = math.sqrt(dx ** 2 + dy ** 2)

            msg = Twist()
            msg.linear.x = 1.5 * distance
            msg.angular.z = 4.0 * math.atan2(dy, dx)
            self.publisher.publish(msg)

            pose_msg = Pose()
            pose_msg.x = trans.transform.translation.x
            pose_msg.y = trans.transform.translation.y
            pose_msg.theta = distance
            self.current_target_pub.publish(pose_msg)

            # Автоматическое переключение
            if distance < self.switch_threshold:
                self.switch_to_next_target()

            # Ручное переключение (если интерактивный терминал)
            key = get_key()
            if key == 'n':
                self.switch_to_next_target()

        except (LookupException, ConnectivityException, ExtrapolationException):
            pass

    def switch_to_next_target(self):
        self.current_target_index = (self.current_target_index + 1) % len(self.targets)
        self.current_target = self.targets[self.current_target_index]
        self.get_logger().info(f"🔁 Переключено на цель: {self.current_target}")

# ------------------------------------------------------------
# 🔹 Точка входа
# ------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    