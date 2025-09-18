cmake_minimum_required(VERSION 3.0.2)
project(frk_act_unified)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  franka_msgs
  franka_gripper
  actionlib_msgs
  tf2_ros
  tf2_geometry_msgs
  gazebo_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    franka_msgs
    franka_gripper
    actionlib_msgs
    tf2_ros
    tf2_geometry_msgs
    gazebo_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(
  PROGRAMS 
    scripts/franka_grasping_controller.py
    scripts/franka_position_grasping_controller.py
    scripts/franka_cartesian_grasping_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
