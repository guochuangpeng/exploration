cmake_minimum_required(VERSION 2.8.12)
project(attitude_controller)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  tf
  mavros
  mavros_extras
  mavros_msgs
  mavlink
  quadrotor_msgs
)

generate_dynamic_reconfigure_options(
  cfg/AttitudeController.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES attitude_controller
  CATKIN_DEPENDS roscpp rospy std_msgs mavros_msgs geometry_msgs sensor_msgs quadrotor_msgs tf
)

#############
# LIBRARIES #
#############
include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/attitude_controller.cpp 
)
add_dependencies(attitude_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

############
# BINARIES #
############
add_executable(attitude_controller_node
  src/attitude_controller_node.cpp
)
add_dependencies(attitude_controller_node attitude_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(attitude_controller_node ${PROJECT_NAME} ${catkin_LIBRARIES})


add_executable(save_data_node src/save_data_node.cpp)
target_link_libraries(save_data_node ${PROJECT_NAME} ${catkin_LIBRARIES})