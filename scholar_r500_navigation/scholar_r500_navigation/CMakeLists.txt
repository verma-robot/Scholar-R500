cmake_minimum_required(VERSION 3.0.2)
project(scholar_r500_navigation)

add_compile_options(-std=c++11)

find_package(catkin  REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  teb_local_planner
  tf
  ydlidar
  yocs_velocity_smoother
  laser_geometry
  pointcloud_to_laserscan
)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
 include  ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(dual_laser
               src/dual_laser_listener.cpp
 )
add_dependencies(dual_laser ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(dual_laser ${catkin_LIBRARIES} ${PCL_LIBRARIES})
