cmake_minimum_required(VERSION 3.14)
project(goal_pose_router)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(goal_pose_router
  src/goal_pose_router.cpp
)

ament_auto_package(
    INSTALL_TO_SHARE 
    launch
)
