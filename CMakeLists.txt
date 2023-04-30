cmake_minimum_required(VERSION 3.0.2)
project(particle_filter)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  tf2
  tf2_msgs
  tf2_ros
  nav_msgs
  )
  
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
    utils
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    tf2
    tf2_msgs
    tf2_ros
    nav_msgs
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(utils
  src/utils.cpp
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
)

target_link_libraries(utils
  ${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${YAML_CPP_LIBRARIES}
  utils
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  utils
)

set(CMAKE_BUILD_TYPE Debug)
