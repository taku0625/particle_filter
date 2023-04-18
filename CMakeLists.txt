cmake_minimum_required(VERSION 3.0.2)
project(particle_filter)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  tf2_msgs
  nav_msgs
)
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(basic_simple_talker   src/basic_simple_talker.cpp)
add_executable(basic_simple_listener src/basic_simple_listener.cpp)

target_link_libraries(basic_simple_talker
  ${catkin_LIBRARIES}
)
target_link_libraries(basic_simple_listener
  ${catkin_LIBRARIES}
)