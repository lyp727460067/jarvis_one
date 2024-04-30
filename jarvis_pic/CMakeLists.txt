cmake_minimum_required(VERSION 3.0.2)
project(jarvis_pic)

add_compile_definitions(GLOG_USE_GLOG_EXPORT)

set(BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE ${BUILD_TYPE})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -fPIC -Ofast -Wno-unused-but-set-variable -Wno-maybe-uninitialized") #-Wno-unused-result

set(EIGEN3_INCLUDE_DIR ${TOP_CMAKE_DIR}/libs/eigen3)
set(OpenCV_INCLUDE_DIRS ${TOP_CMAKE_DIR}/libs/opencv-4.6.0/include/opencv4)
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


set(CERES_INCLUDE_DIRS ${TOP_CMAKE_DIR}/third_party/ceres-solver/include/)
set(GLOG_INCLUDE_DIRS ${TOP_CMAKE_DIR}/third_party/glog/include)
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

include_directories(
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  ${GLOG_INCLUDE_DIRS}
  ${TOP_CMAKE_DIR}/libs/libmpc/include
)

add_executable(${PROJECT_NAME}_pic_main  jarvis_pic_main)
add_executable(${PROJECT_NAME}_main  jarvis_main.cc data_capture.cc)
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_pic_main PUBLIC
  glog
  jarvis
  ${OpenCV_LIBS}
  -ldl
)
target_link_libraries(${PROJECT_NAME}_main PUBLIC
  glog
  jarvis
  ${OpenCV_LIBS}
  -ldl
  mpc
)
