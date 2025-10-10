cmake_minimum_required(VERSION 3.8)
project(action_cleaning_robot)

# Используем Python и ROS2
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Генерация Python сообщений для action
rosidl_generate_interfaces(${PROJECT_NAME}
  "/Users/andrejtiskin/ros2_ws/src/action_cleaning_robot/action/CleaningTask.action"
)
#rosidl_generate_interfaces(${PROJECT_NAME}
#  "action/CleaningTask.action"
#)

# Установка Python пакета
ament_python_install_package(${PROJECT_NAME})

# Установка скриптов (ros2 run)
install(PROGRAMS
  action_cleaning_robot/cleaning_action_server.py
  action_cleaning_robot/cleaning_action_client.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()