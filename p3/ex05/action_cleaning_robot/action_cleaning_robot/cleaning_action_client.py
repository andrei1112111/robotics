import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_cleaning_robot_interfaces.action import CleaningTask

class CleaningActionClient(Node):
    def __init__(self):
        super().__init__('cleaning_action_client')
        self.client = ActionClient(self, CleaningTask, 'cleaning_task')

    def send_goal_and_wait(self, task_type, area_size, x=0.0, y=0.0):
        goal = CleaningTask.Goal()
        goal.task_type = task_type
        goal.area_size = area_size
        goal.target_x = x
        goal.target_y = y
        self.get_logger().info(f"Sending task: {task_type}")
        self.client.wait_for_server()
        send_future = self.client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return
        self.get_logger().info("Goal accepted!")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(
            f"Result: success={result.success}, cleaned={result.cleaned_points}"
        )

    def feedback_callback(self, fb_msg):
        fb = fb_msg.feedback
        self.get_logger().info(
            f"Progress {fb.progress_percent}% at ({fb.current_x:.2f}, {fb.current_y:.2f})"
        )

def main():
    rclpy.init()
    node = CleaningActionClient()
    # goals идут ПО ОЧЕРЕДИ 
    # 5.5, 5.5 как в примере задания и clean_square 3.0
    node.send_goal_and_wait("return_home", 0.0, 5.5, 5.5)
    node.send_goal_and_wait("clean_square", 3.0)
    node.send_goal_and_wait("return_home", 0.0, 5.5, 5.5)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
