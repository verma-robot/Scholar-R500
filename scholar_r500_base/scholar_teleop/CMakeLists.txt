cmake_minimum_required(VERSION 2.8.3)
project(scholar_teleop)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
)


catkin_package()
include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(scholar_teleop_bringup
               src/scholar_teleop_bringup.cpp
               src/scholar_teleop.cpp
 )
add_dependencies(scholar_teleop_bringup ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(scholar_teleop_bringup ${catkin_LIBRARIES} ${Boost_LIBRARIES})
