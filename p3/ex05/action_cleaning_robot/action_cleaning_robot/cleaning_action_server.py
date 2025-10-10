import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from rclpy.action import ActionServer
from action_cleaning_robot.action import CleaningTask
import math
import time

class CleaningActionServer(Node):

    def __init__(self):
        super().__init__('cleaning_action_server')
        self._action_server = ActionServer(
            self,
            CleaningTask,
            'cleaning_action',
            self.execute_callback
        )

        self.cmd_pub = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.current_pose = Pose()

    def pose_callback(self, msg):
        self.current_pose = msg

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Received goal: {goal_handle.request.task_type}')
        goal = goal_handle.request

        result = CleaningTask.Result()
        feedback = CleaningTask.Feedback()
        cleaned_points = 0
        total_distance = 0.0

        # Simple simulation: just move forward in steps and update feedback
        if goal.task_type in ['clean_square', 'clean_circle']:
            steps = int(goal.area_size * 10)  # arbitrary resolution
            for i in range(steps):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')
                    return CleaningTask.Result()

                # send forward velocity
                twist = Twist()
                twist.linear.x = 1.0
                self.cmd_pub.publish(twist)

                # update feedback
                cleaned_points += 1
                total_distance += 0.1
                feedback.progress_percent = int((i + 1) / steps * 100)
                feedback.current_cleaned_points = cleaned_points
                feedback.current_x = self.current_pose.x
                feedback.current_y = self.current_pose.y
                goal_handle.publish_feedback(feedback)
                time.sleep(0.1)

        elif goal.task_type == 'return_home':
            target_x, target_y = goal.target_x, goal.target_y
            while math.hypot(self.current_pose.x - target_x, self.current_pose.y - target_y) > 0.1:
                twist = Twist()
                angle = math.atan2(target_y - self.current_pose.y, target_x - self.current_pose.x)
                twist.linear.x = 1.5
                twist.angular.z = angle - self.current_pose.theta
                self.cmd_pub.publish(twist)

                feedback.current_x = self.current_pose.x
                feedback.current_y = self.current_pose.y
                goal_handle.publish_feedback(feedback)
                time.sleep(0.1)

        # stop turtle
        self.cmd_pub.publish(Twist())

        result.success = True
        result.cleaned_points = cleaned_points
        result.total_distance = total_distance
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = CleaningActionServer()
    rclpy.spin(node)

if __name__ == '__main__':
    main()