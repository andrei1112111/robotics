import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from action_cleaning_robot_interfaces.action import CleaningTask
import math
import time
import copy

class CleaningActionServer(Node):
    def __init__(self):
        super().__init__('cleaning_action_server')
        self.pose = None
        self.prev_pose = None
        self.active_goal = None
        self.goal_handle = None
        self.total_distance = 0.0
        self.cleaned_points = 0
        self.visited = set()
        self.total_cells = 0
        self.grid_res = 0.2
        self.clean_area = None
        self.waypoints = []
        self.current_wp_idx = 0
        self.initial_dist = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.last_feedback_time = 0.0
        self.done = False
        self.result = None
        self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.timer = self.create_timer(0.05, self.control_loop)
        self.action_server = ActionServer(
            self,
            CleaningTask,
            "cleaning_task",
            execute_callback=self.execute_callback
        )
        self.get_logger().info("Cleaning Action Server started.")

    def pose_callback(self, msg):
        self.pose = msg

    def normalize(self, a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a

    def publish_cmd(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Received goal: {goal_handle.request.task_type}")
        self.active_goal = goal_handle.request
        self.goal_handle = goal_handle
        self.total_distance = 0.0
        self.cleaned_points = 0
        self.visited = set()
        self.prev_pose = copy.deepcopy(self.pose) if self.pose else None
        self.last_feedback_time = time.time()
        task = self.active_goal
        if task.task_type == "clean_square":
            self.init_clean_square(task.area_size)
        elif task.task_type == "clean_circle":
            self.init_clean_circle(task.area_size)
        elif task.task_type == "return_home":
            self.init_return_home(task.target_x, task.target_y)
        else:
            result = CleaningTask.Result()
            result.success = False
            goal_handle.abort()
            return result
        self.done = False
        while not self.done:
            time.sleep(0.1)
        goal_handle.succeed()
        return self.result

    def init_clean_square(self, size):
        self.clean_area = "square"
        self.min_x = self.pose.x
        self.min_y = self.pose.y
        self.size = size
        num_cells_side = math.ceil(self.size / self.grid_res)
        self.total_cells = num_cells_side * num_cells_side
        # Build waypoints for lawnmower pattern
        self.waypoints = []
        line_spacing = self.grid_res
        num_lines = math.ceil(self.size / line_spacing)
        direction_right = True
        current_y = self.min_y
        for line in range(num_lines):
            y_line = min(current_y, self.min_y + self.size)
            if direction_right:
                wp_x = self.min_x + self.size
            else:
                wp_x = self.min_x
            self.waypoints.append((wp_x, y_line))
            if line < num_lines - 1:
                next_y = current_y + line_spacing
                next_y = min(next_y, self.min_y + self.size)
                self.waypoints.append((wp_x, next_y))
            current_y += line_spacing
            direction_right = not direction_right
        self.current_wp_idx = 0

    def init_clean_circle(self, radius):
        self.clean_area = "circle"
        self.center_x = self.pose.x
        self.center_y = self.pose.y
        self.size = radius
        # Calculate total_cells exactly
        res = self.grid_res
        min_i = math.floor((self.center_x - radius) / res)
        max_i = math.ceil((self.center_x + radius) / res)
        min_j = math.floor((self.center_y - radius) / res)
        max_j = math.ceil((self.center_y + radius) / res)
        self.total_cells = 0
        for i in range(min_i, max_i):
            for j in range(min_j, max_j):
                cx = i * res + res / 2.0
                cy = j * res + res / 2.0
                if math.hypot(cx - self.center_x, cy - self.center_y) <= radius:
                    self.total_cells += 1
        # Build waypoints for spiral
        self.waypoints = []
        spacing = 0.2
        b = spacing / (2 * math.pi)
        max_theta = (2 * math.pi * radius) / spacing
        dtheta = 0.1
        theta = 0.0
        while theta <= max_theta:
            r = b * theta
            wx = self.center_x + r * math.cos(theta)
            wy = self.center_y + r * math.sin(theta)
            self.waypoints.append((wx, wy))
            theta += dtheta
        self.current_wp_idx = 0

    def init_return_home(self, x_goal, y_goal):
        self.target_x = x_goal
        self.target_y = y_goal
        self.initial_dist = math.hypot(self.pose.x - x_goal, self.pose.y - y_goal)
        self.waypoints = [(x_goal, y_goal)]
        self.current_wp_idx = 0
        self.clean_area = None
        self.total_cells = 0

    def is_in_area(self, x, y):
        if self.clean_area == "square":
            return self.min_x <= x <= self.min_x + self.size and self.min_y <= y <= self.min_y + self.size
        elif self.clean_area == "circle":
            return math.hypot(x - self.center_x, y - self.center_y) <= self.size
        return False

    def send_feedback(self):
        feedback = CleaningTask.Feedback()
        if self.clean_area:
            percent = 100 if self.total_cells == 0 else int(100 * self.cleaned_points / self.total_cells)
        else:  # return_home
            if self.initial_dist > 0:
                current_dist = math.hypot(self.pose.x - self.target_x, self.pose.y - self.target_y)
                percent = int(100 * (1 - current_dist / self.initial_dist))
            else:
                percent = 100
        feedback.progress_percent = max(0, min(100, percent))
        feedback.current_cleaned_points = self.cleaned_points
        feedback.current_x = self.pose.x
        feedback.current_y = self.pose.y
        self.goal_handle.publish_feedback(feedback)
        self.last_feedback_time = time.time()

    def do_task(self):
        if self.current_wp_idx >= len(self.waypoints):
            self.publish_cmd(0.0, 0.0)
            self.result = CleaningTask.Result()
            self.result.success = True
            self.result.cleaned_points = self.cleaned_points
            self.result.total_distance = self.total_distance
            self.get_logger().info(f"{self.active_goal.task_type}: DONE!")
            self.done = True
            self.active_goal = None
            return
        wp_x, wp_y = self.waypoints[self.current_wp_idx]
        dx = wp_x - self.pose.x
        dy = wp_y - self.pose.y
        dist = math.sqrt(dx * dx + dy * dy)
        angle_to = math.atan2(dy, dx)
        angle_err = self.normalize(angle_to - self.pose.theta)
        if abs(angle_err) > 0.1 and dist > 0.1:
            self.publish_cmd(0.0, 2.0 * angle_err)
        elif dist > 0.1:
            self.publish_cmd(min(1.5 * dist, 2.0), 1.5 * angle_err)
        else:
            self.current_wp_idx += 1

    def control_loop(self):
        if self.pose is None or self.active_goal is None:
            return
        # Update total distance
        if self.prev_pose is not None:
            dx = self.pose.x - self.prev_pose.x
            dy = self.pose.y - self.prev_pose.y
            self.total_distance += math.sqrt(dx * dx + dy * dy)
        self.prev_pose = copy.deepcopy(self.pose)
        # Update visited cells if cleaning
        if self.clean_area and self.is_in_area(self.pose.x, self.pose.y):
            cell = (math.floor(self.pose.x / self.grid_res), math.floor(self.pose.y / self.grid_res))
            if cell not in self.visited:
                self.visited.add(cell)
                self.cleaned_points += 1
        # Send feedback periodically
        if time.time() - self.last_feedback_time > 0.5:
            self.send_feedback()
        # Perform task control
        self.do_task()

def main():
    rclpy.init()
    node = CleaningActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()