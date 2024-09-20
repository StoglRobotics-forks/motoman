cmake_minimum_required(VERSION 3.8)
project(motoman_msgs)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(industrial_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)


find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DynamicJointPoint.msg"
  "msg/DynamicJointsGroup.msg"
  "msg/DynamicJointState.msg"
  "msg/DynamicJointTrajectory.msg"
  "msg/DynamicJointTrajectoryFeedback.msg"
  "srv/CmdJointTrajectoryEx.srv"
  "srv/ReadMRegister.srv"
  "srv/ReadSingleIO.srv"
  "srv/ReadGroupIO.srv"
  "srv/SelectTool.srv"
  "srv/WriteMRegister.srv"
  "srv/WriteSingleIO.srv"
  "srv/WriteGroupIO.srv"
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    control_msgs
    industrial_msgs
    trajectory_msgs
 )

if(BUILD_TESTING)
endif()

ament_package()
