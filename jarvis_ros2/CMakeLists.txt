cmake_minimum_required(VERSION 3.5)
project(jarvis_ros2)

set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
ament_package()




## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
## System dependencies are found with CMake's conventions
set(BOOST_COMPONENTS iostreams filesystem)



## Specify additional locations of header files
## Your package locations should be listed before other locations
# message(${loopdetection_INCLUDE_DIRS})
find_package(Glog REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTest REQUIRED)
find_package(jarvis REQUIRED)
#
find_package(OpenCV REQUIRED) #version 4.6.0

include_directories(
${OpenCV_INCLUDE_DIRS}
)


## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/LoopDetection_ros.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_main  jarvis_main.cc ros_component.cc
)
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_main
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  glog::glog
  yaml-cpp
  jarvis
)
ament_target_dependencies(${PROJECT_NAME}_main rclcpp std_msgs nav_msgs sensor_msgs
visualization_msgs
tf2_ros
cv_bridge
image_transport
)


