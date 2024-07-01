cmake_minimum_required(VERSION 3.0.2)
project(quadrotor_control)

find_package(catkin REQUIRED COMPONENTS
  mavros_msgs
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS mavros_msgs roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_lib src/quadrotor_control.cpp)

add_executable(${PROJECT_NAME}_node src/main.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_lib
)
