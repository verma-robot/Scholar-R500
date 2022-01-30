cmake_minimum_required(VERSION 3.0.2)
project(scholar_r500_imu_bringup)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(scholar_r500_imu_driver
               src/imu_driver.cpp
               src/imu.cpp
 )
add_dependencies(scholar_r500_imu_driver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(scholar_r500_imu_driver ${catkin_LIBRARIES} ${Boost_LIBRARIES})


