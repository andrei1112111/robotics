import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_cleaning_robot.action import CleaningTask

class CleaningActionClient(Node):
    def __init__(self):
        super().__init__('cleaning_action_client')
        self._client = ActionClient(self, CleaningTask, 'cleaning_action')

    def send_goal(self, task_type, area_size=0.0, target_x=0.0, target_y=0.0):
        self._client.wait_for_server()
        goal_msg = CleaningTask.Goal()
        goal_msg.task_type = task_type
        goal_msg.area_size = area_size
        goal_msg.target_x = target_x
        goal_msg.target_y = target_y

        future = self._client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Progress: {feedback.progress_percent}% | Cleaned: {feedback.current_cleaned_points}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: success={result.success}, cleaned_points={result.cleaned_points}, total_distance={result.total_distance}')

def main(args=None):
    rclpy.init(args=args)
    client = CleaningActionClient()

    # example sequence: clean square 3x3, then return home
    client.send_goal('clean_square', area_size=3.0)
    rclpy.spin(client)

if __name__ == '__main__':
    main()