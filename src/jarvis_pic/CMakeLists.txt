cmake_minimum_required(VERSION 3.0.2)
# include(${TOP_CMAKE_DIR}/vslam/jarvis)
project(jarvis_pic)

add_compile_definitions(GLOG_USE_GLOG_EXPORT)

set_target_properties(jarvis PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "")
set(BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE ${BUILD_TYPE})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -O3 -s -Wno-unused-but-set-variable -Wno-maybe-uninitialized") #-Wno-unused-result
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
set(Opencv_lib_dir ${TOP_CMAKE_DIR}/third_party/opencv-4.6.0/lib/mr527)
set(OpenCV_LIBS
    ${Opencv_lib_dir}/libopencv_core.so            # ${OpenCV_LIBRARIES}
    ${Opencv_lib_dir}/libopencv_imgcodecs.so
    ${Opencv_lib_dir}/libopencv_imgproc.so
    ${Opencv_lib_dir}/libopencv_features2d.so
    ${Opencv_lib_dir}/libopencv_calib3d.so
    ${Opencv_lib_dir}/libopencv_dnn.so
    ${Opencv_lib_dir}/libopencv_flann.so
    ${Opencv_lib_dir}/libopencv_video.so
)
# 手动指定 OpenCL 的位置
set(OPENCL_INCLUDE_DIRS ${TOP_CMAKE_DIR}/toolchains/mr527-aarch64-gcc-12.3/usr/include)
# set(OPENCL_LIBRARIES ${TOP_CMAKE_DIR}/third_party/opencl/lib/libOpenCL.so)

set(EIGEN3_INCLUDE_DIR ${TOP_CMAKE_DIR}/third_party/eigen-3.4.0)
set(OpenCV_INCLUDE_DIRS ${TOP_CMAKE_DIR}/third_party/opencv-4.6.0/include/opencv4)
set(CERES_INCLUDE_DIRS ${TOP_CMAKE_DIR}/third_party/ceres-solver/include/)
set(GLOG_INCLUDE_DIRS ${TOP_CMAKE_DIR}/third_party/glog/include)
set(YAML_INCLUDE_DIRS ${TOP_CMAKE_DIR}/third_party/yaml-cpp/include)
set(OPENCL_LIBRARIES ${TOP_CMAKE_DIR}/toolchains/mr527-aarch64-gcc-12.3/usr/lib/libmali.so)
# set(glog_binary_dir ${TOP_CMAKE_DIR}/third_party/glog/lib/libglog.a)
# add_library(
#         glog
#         STATIC
#         IMPORTED
#         GLOBAL
# )
# set_target_properties(
#         glog
#         PROPERTIES IMPORTED_LOCATION
#         ${glog_binary_dir}
# )
get_directory_property(out_var INCLUDE_DIRECTORIES)
message(****${out_var})
string(REPLACE  "eigen3" "eigen5" new_include_dirs ${out_var})
set_directory_properties(DIRECTORY  PROPERTY  jarvis INCLUDE_DIRECTORIES ${new_include_dirs})
get_directory_property(out_var1 INCLUDE_DIRECTORIES)
message(********${out_var1})

INCLUDE_DIRECTORIES(
  ${TOP_CMAKE_DIR}/vslam/src/jarvis/
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${YAML_INCLUDE_DIRS}/
  ${GLOG_INCLUDE_DIRS}
  ${TOP_CMAKE_DIR}/libs/libmpc/include/
  ${TOP_CMAKE_DIR}/libs/libimageproc/include/
  ${TOP_CMAKE_DIR}/libs/mpsdk/
)

option(view_debug " view_debug" ON)
if(${view_debug})
add_definitions(-D__ZMQ_ENABLAE__)
endif()
# add_executable(${PROJECT_NAME}_pic_main  jarvis_pic_main.cc)
add_executable(mpslam  jarvis_slipe_detect_main.cc  
 data_capture.cc zmq_component.cc dev_socket.cc 
 data_record.cc
 socket_wrapper.cc jarvis_brige.cc
 glog_sink.cc
 gpu_pyramid_compute.cc
 opencl_handler.cpp
 )
add_executable(jarvis_eigen_test  jarvis_eigen_test.cc
 )


target_link_libraries(jarvis_eigen_test PUBLIC
  -ldl
)



add_executable(${PROJECT_NAME}_gpu_main  jarvis_gpu_compute_py_test_main.cc

)
# Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_gpu_main PUBLIC
  # glog
  ${OPENCL_LIBRARIES}
  ${OpenCV_LIBS}
  -ldl
)


target_include_directories( ${PROJECT_NAME}_gpu_main  SYSTEM PRIVATE   ${EIGEN3_INCLUDE_DIR})
target_link_libraries(mpslam PUBLIC
  # glog
  jarvis
  ${OpenCV_LIBS}
  -ldl
   mpc

)

target_include_directories(mpslam  SYSTEM PRIVATE   ${EIGEN3_INCLUDE_DIR})
target_link_libraries(mpslam PUBLIC
  jarvis
  ${OpenCV_LIBS}
  -ldl
  mpc
  ${OPENCL_LIBRARIES}
)
target_include_directories(mpslam PUBLIC jarvis )