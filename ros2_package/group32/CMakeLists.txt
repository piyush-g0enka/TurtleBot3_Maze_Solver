cmake_minimum_required(VERSION 3.8)
project(group32)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# # find dependencies
# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(geometry_msgs REQUIRED) # Add this if you use geometry_msgs


include_directories(include/group32)


# Create a variable for all dependencies
set(FRAME_DEMO_INCLUDE_DEPENDS
  rclcpp 
  tf2_ros
  geometry_msgs
  tf2
  tf2_geometry_msgs
  ros2_aruco_interfaces
  nav_msgs
  mage_msgs
)

# Find all dependencies
foreach(dependency IN ITEMS ${FRAME_DEMO_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

#-----------------------------
# C++
#-----------------------------

add_executable(aruco_broadcaster src/aruco_broadcaster.cpp src/utils.cpp)
ament_target_dependencies(aruco_broadcaster
  ${FRAME_DEMO_INCLUDE_DEPENDS}
)

add_executable(parts_broadcaster src/parts_broadcaster.cpp src/utils.cpp)
ament_target_dependencies(parts_broadcaster
  ${FRAME_DEMO_INCLUDE_DEPENDS}
)

add_executable(robot_controller src/robot_controller.cpp src/utils.cpp)
ament_target_dependencies(robot_controller
  ${FRAME_DEMO_INCLUDE_DEPENDS}
)



#-----------------------------
# Installation
#-----------------------------

# Install the executable
install(TARGETS
  robot_controller
  parts_broadcaster
  aruco_broadcaster
  DESTINATION lib/${PROJECT_NAME}
)

#

#nstall(DIRECTORY include
#  DESTINATION share/${PROJECT_NAME}/
#)



# Install the "config" folder
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install the "launch" folder
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
