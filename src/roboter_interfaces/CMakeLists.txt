cmake_minimum_required(VERSION 3.8)
project(roboter_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Antriebsparameter.msg"
  "msg/Armkommando.msg"
  "msg/Handkommando.msg"
  "action/GehAktion.action"
  "action/BalancierAktion.action"
  "action/GreifAktion.action"
  "srv/GleichgewichtService.srv"
  "srv/MissionStarten.srv"
)

ament_package()
