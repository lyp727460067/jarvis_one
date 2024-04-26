cmake_minimum_required(VERSION 3.0)
project(jarvis VERSION 1.0.0)

add_compile_definitions(GLOG_USE_GLOG_EXPORT)

set(BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE ${BUILD_TYPE})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -fPIC -Ofast -Wno-unused-but-set-variable -Wno-maybe-uninitialized") #-Wno-unused-result


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

add_compile_options(#-Werror#-Wpedantic #minor error
                   -Wreturn-type # warn no return 
                   -Wuninitialized # uninitlialized varibales
                  # -Wshadow # shadow local variable
                  #  -Werror=unused-but-set-variable
                   -Wmultistatement-macros)


option(build_test "build test" ON)
set(EIGEN3_INCLUDE_DIR ${TOP_CMAKE_DIR}/libs/eigen3)
set(OpenCV_INCLUDE_DIRS ${TOP_CMAKE_DIR}/libs/opencv-4.6.0/include/opencv4)
set(CERES_INCLUDE_DIRS ${TOP_CMAKE_DIR}/third_party/ceres-solver/include/)
set(CERES_LIBRARIES ${TOP_CMAKE_DIR}/third_party/ceres-solver/lib/libceres.a)
set(Opencv_lib_dir ${TOP_CMAKE_DIR}/third_party/opencv-4.6.0/lib/mr527)
# find_package(Eigen3 3.4.0 EXACT) #version 3.4.0
# lie group and lie algebra
# find_package(OpenCV REQUIRED) #version 4.6.0
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

set(GLOG_INCLUDE_DIRS ${TOP_CMAKE_DIR}/third_party/glog/include)
set(glog_binary_dir ${TOP_CMAKE_DIR}/third_party/glog/lib/libglog.a)
add_library(
        glog
        STATIC
        IMPORTED
        GLOBAL
)
set_target_properties(
        glog
        PROPERTIES IMPORTED_LOCATION
        ${glog_binary_dir}
)


# ceres optimization replace gtsam later
# find_package(Ceres REQUIRED)
# find_package(Glog REQUIRED) #version 0.6.0
# test system
#
##
##add proto
# find_package(Protobuf  REQUIRED)
# configure_file(src/mapping/test_constant.h.in ${PROJECT_BINARY_DIR}/mapping/test_constant.h)
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/jarvis
  ${PROJECT_SOURCE_DIR}/
  ${Boost_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
)

##
file(GLOB_RECURSE ALL_LIBRARY_HDRS "jarvis/*.hpp" "jarvis/*.h" )
file(GLOB_RECURSE ALL_LIBRARY_SRCS "jarvis/*.cc" "jarvis/*.cpp")
##
file(GLOB_RECURSE OJBECT_LIBRARY_SRCS "jarvis/object*.cc")
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${OJBECT_LIBRARY_SRCS})
##
# file(GLOB_RECURSE ALL_TEST_SRCS  "src/*_test.cc" "src/*_test.cpp")
add_library(${PROJECT_NAME} SHARED ${ALL_LIBRARY_SRCS}  )
target_link_libraries( ${PROJECT_NAME} PUBLIC ${PROTOBUF_INCLUDE_LIB}  )
target_link_libraries(
  ${PROJECT_NAME} PUBLIC
  glog
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROTOBUF_INCLUDE_DIRS})

#
if(${build_test})
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
      ${PROJECT_NAME}
    )
   endforeach()
endif()


target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${PROJECT_BINARY_DIR}>
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT JarvisExport
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

set(JARVIS_CMAKE_DIR share/jarvis/cmake)
include(CMakePackageConfigHelpers)
configure_package_config_file(
  jarvis-config.cmake.in
  ${PROJECT_BINARY_DIR}/cmake/jarvis/jarvis-config.cmake
  PATH_VARS JARVIS_CMAKE_DIR
  INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/share/jarvis
)

install(
  EXPORT JarvisExport
  DESTINATION share/jarvis/cmake/
  FILE JarvisTargets.cmake
)

install(
  FILES ${PROJECT_BINARY_DIR}/cmake/jarvis/jarvis-config.cmake
  DESTINATION share/jarvis/
)




file(GLOB_RECURSE INSTALL_SOURCE_HDRS "jarvis/*.h"
)

foreach(HDR ${INSTALL_SOURCE_HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
    FILES ${HDR}
    DESTINATION  include/${DIR}
  )
endforeach()