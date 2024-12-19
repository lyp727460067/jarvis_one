cmake_minimum_required(VERSION 3.5)
project(jarvis_ros2)

set(CMAKE_CXX_STANDARD 14)

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()
set(BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE ${BUILD_TYPE})
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

find_package(OpenCV REQUIRED COMPONENTS  core imgproc calib3d video imgcodecs highgui) 
find_package(Protobuf  REQUIRED) 

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_BINARY_DIR}/../jarvis/
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
# add_executable(${PROJECT_NAME}_main  jarvis_main.cc ros_component.cc
# )
add_executable(${PROJECT_NAME}_zmp_main  jarvis_zmp_main.cc)
add_executable(ld_${PROJECT_NAME}_slip_main   jarvis_slip_detect_main.cc ros_component.cc
)
add_executable(ld_${PROJECT_NAME}_pose_compare_main  jarvis_rtk_vio_pose_compare.cc pose_optimization.cc
)

add_executable(ld_${PROJECT_NAME}_data_parse  jarvis_data_parse.cc
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
# target_link_libraries(${PROJECT_NAME}_main
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
#   glog::glog
#   yaml-cpp
#   jarvis
# )

target_link_libraries(ld_${PROJECT_NAME}_slip_main
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  glog::glog
  yaml-cpp
  jarvis
  ${OpenCV_LIBS}
)
target_link_libraries(ld_${PROJECT_NAME}_data_parse
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  glog::glog
  yaml-cpp
  jarvis
)

target_link_libraries(ld_${PROJECT_NAME}_pose_compare_main
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  glog::glog
  yaml-cpp
  jarvis
)
set(ALL_TEST_SRCS  ${PROJECT_SOURCE_DIR}/dbow_test/vocabulary_test.cc
${PROJECT_SOURCE_DIR}/dbow_test/to_dbow2_vocabulary_adptor_test.cc
)

# add_subdirectory(${PROJECT_SOURCE_DIR}/dbow_test/DBow2/DBoW2)
file(GLOB_RECURSE ALL_PROTO_LIBRARY_SRCS "dbow_test/*.cc" "dbow_test/*.cpp")

add_library(dbow_test ${ALL_PROTO_LIBRARY_SRCS})
target_link_libraries(dbow_test
  ${Boost_LIBRARIES}
  glog::glog
  ${PROTOBUF_LIBRARIES}
  jarvis
)

target_include_directories(dbow_test PUBLIC  ${PROJECT_SOURCE_DIR}/dbow_test/DBow2/DBoW2/include/DBoW2/)
target_include_directories(dbow_test PUBLIC  ${PROJECT_SOURCE_DIR}/dbow_test/DBow2/DBoW2/include/)
foreach(ABS_FIL ${ALL_TEST_SRCS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
  string(REPLACE "/" "." TARGET_NAME "${DIR}/${FIL_WE}")
  add_executable(${TARGET_NAME} ${ABS_FIL} )
  target_link_libraries(${TARGET_NAME}
    -lgtest_main
    -lgtest
    -lpthread
    jarvis
    ${OpenCV_LIBS}
    dbow_test
  )
  target_include_directories(${TARGET_NAME} PUBLIC  ${PROJECT_SOURCE_DIR}/dbow_test/DBow2/DBoW2/include/)
 endforeach()



ament_target_dependencies(ld_${PROJECT_NAME}_pose_compare_main rclcpp std_msgs nav_msgs sensor_msgs
visualization_msgs
tf2_ros
cv_bridge
)

ament_target_dependencies(ld_${PROJECT_NAME}_data_parse rclcpp std_msgs nav_msgs sensor_msgs
visualization_msgs
tf2_ros
cv_bridge
image_transport
)

ament_target_dependencies(ld_${PROJECT_NAME}_slip_main rclcpp std_msgs nav_msgs sensor_msgs
visualization_msgs
tf2_ros
cv_bridge
image_transport
)
# ament_target_dependencies(${PROJECT_NAME}_main rclcpp std_msgs nav_msgs sensor_msgs
# visualization_msgs
# tf2_ros
# cv_bridge
# image_transport
# )
target_link_libraries(${PROJECT_NAME}_zmp_main
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  glog::glog
  yaml-cpp
  jarvis
)
ament_target_dependencies(${PROJECT_NAME}_zmp_main rclcpp std_msgs nav_msgs sensor_msgs
visualization_msgs
tf2_ros
cv_bridge
image_transport
)


