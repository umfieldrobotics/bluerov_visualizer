cmake_minimum_required(VERSION 3.0.2)
project(bluerov_visualizer)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  ping_sonar
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  waterlinked_a50_ros_driver
  bluerov_state_estimation_online
  bluerov_mapping_online
  message_filters
  pcl_ros
  cv_bridge
)

catkin_package(
 CATKIN_DEPENDS geometry_msgs nav_msgs ping_sonar roscpp rospy sensor_msgs std_msgs tf waterlinked_a50_ros_driver stereo_msgs cv_bridge message_filters pcl_ros
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_executable(depth_visualizer_node src/depth_visualizer.cpp)
# set_target_properties(depth_visualizer_node PROPERTIES OUTPUT_NAME node PREFIX "")
add_dependencies(depth_visualizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(depth_visualizer_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
)
catkin_install_python(PROGRAMS 
        scripts/visualize_acoustics.py
        scripts/visualize_depth.py
          DESTINATION $(CATKIN_PACKAGE_BIN_DESTINATION))