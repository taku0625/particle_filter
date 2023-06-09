cmake_minimum_required(VERSION 3.0.2)
project(particle_filter)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-std=c++14 -O2 -g -Wall)

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
    likelihood_field_model
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

add_library(${PROJECT_NAME}_utils
  src/utils.cpp
)

add_library(likelihood_field_model
  src/likelihood_field_model.cpp
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
)

target_link_libraries(${PROJECT_NAME}_utils
  ${catkin_LIBRARIES}
)

target_link_libraries(likelihood_field_model
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${YAML_CPP_LIBRARIES}
  likelihood_field_model
  ${PROJECT_NAME}_utils
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  likelihood_field_model
  ${PROJECT_NAME}_utils
)

set(CMAKE_BUILD_TYPE Debug)
