cmake_minimum_required(VERSION 3.8)
project(yaskawa_ros2_control_support)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_manager
  rclcpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(ros2_control_node_steady_clock src/ros2_control_node_steady_clock.cpp)
target_include_directories(ros2_control_node_steady_clock PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/yaskawa_ros2_control_support>
)
ament_target_dependencies(ros2_control_node_steady_clock PUBLIC ${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS ros2_control_node_steady_clock
  RUNTIME DESTINATION lib/yaskawa_ros2_control_support
)


install(
  DIRECTORY config urdf launch
  DESTINATION share/yaskawa_ros2_control_support
)


ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
