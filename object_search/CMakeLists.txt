cmake_minimum_required(VERSION 3.0.2)
project(object_search)

find_package(catkin REQUIRED COMPONENTS
  dmce_msgs
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  std_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES object_search
#  CATKIN_DEPENDS dmce_msgs geometry_msgs nav_msgs rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS
  scripts/keyboard_teleop.py
  # scripts/door_targets_node.py
  scripts/simulated_lidar_node.py
  scripts/lidar_node_test.py
  scripts/mapupdate_to_cloud.py
  scripts/vel_key_node.py
  scripts/object_detector_node.py
  scripts/door_explorer_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(hello_node src/hello_node.cpp)
target_link_libraries(hello_node ${catkin_LIBRARIES})
