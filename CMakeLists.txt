cmake_minimum_required(VERSION 3.0.2)
project(ur5_picknplace)


find_package(catkin REQUIRED
  COMPONENTS
    interactive_markers
    moveit_core
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_perception
    rviz_visual_tools
    moveit_visual_tools
    pluginlib
    geometric_shapes
    tf2_ros
    tf2_geometry_msgs
    Eigen3
)

include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
)

add_executable(picknplace src/picknplace.cpp)
target_link_libraries(picknplace ${catkin_LIBRARIES})

add_executable(planning_scene src/planning_scene.cpp)
target_link_libraries(planning_scene ${catkin_LIBRARIES})