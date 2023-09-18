cmake_minimum_required(VERSION 3.12)

project(motoman_gp250_support)

find_package(ament_cmake REQUIRED)
ament_package()

install(DIRECTORY meshes urdf launch
  DESTINATION share/${PROJECT_NAME})
