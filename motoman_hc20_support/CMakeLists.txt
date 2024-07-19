cmake_minimum_required(VERSION 3.12)

project(motoman_hc20_support)

find_package(ament_cmake REQUIRED)


install(DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME})

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")

ament_package()