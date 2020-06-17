cmake_minimum_required(VERSION 2.8.3)
project(social_sim_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_message_files(
  FILES
    PersonEntry.msg
    PersonEntryArray.msg
)

find_package(catkin REQUIRED COMPONENTS
    controller_interface
    controller_manager
    control_msgs
    nav_msgs
    realtime_tools
    tf
    urdf
    pluginlib
    message_generation
    std_msgs
    geometry_msgs
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    message_runtime
    std_msgs
    geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(differential_drive_sim_controller src/differential_drive_sim_controller.cpp)
target_link_libraries(differential_drive_sim_controller ${catkin_LIBRARIES})
