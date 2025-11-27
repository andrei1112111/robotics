cmake_minimum_required(VERSION 3.5)
project(action_cleaning_robot_interfaces)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CleaningTask.action"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
