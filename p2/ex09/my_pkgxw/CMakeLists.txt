cmake_minimum_required(VERSION 3.5)
project(my_pkgxw)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FullNameMessage.msg"
  "srv/FullNameSumService.srv"
)

ament_package()

