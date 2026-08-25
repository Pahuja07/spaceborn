cmake_minimum_required(VERSION 3.20)
project(clearpath_control)


find_package(ament_cmake REQUIRED)


install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/twist_stamper DESTINATION lib/${PROJECT_NAME})
ament_package()