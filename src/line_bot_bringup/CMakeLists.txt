cmake_minimum_required(VERSION 3.5)
project(line_bot_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  worlds
  DESTINATION share/${PROJECT_NAME})

install(FILES
  resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages)

ament_package()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
