cmake_minimum_required(VERSION 3.5)
project(line_bot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME})

install(FILES
  resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages)

ament_package()
