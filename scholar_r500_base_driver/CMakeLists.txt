cmake_minimum_required(VERSION 3.0.2)
project(scholar_r500_base_driver)

 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  roslint
  rospy
  sensor_msgs
  std_msgs
  tf
  message_generation
)


  add_message_files(
   FILES
   scholar_r500_battery.msg
  )

 generate_messages(
   DEPENDENCIES std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(scholar_r500_bringup
               src/scholar_r500_bringup.cpp
               src/scholar_r500_driver.cpp
 )
add_dependencies(scholar_r500_bringup ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 
target_link_libraries(scholar_r500_bringup ${catkin_LIBRARIES} ${Boost_LIBRARIES})
