cmake_minimum_required(VERSION 3.0)
project(jarvis VERSION 1.0.0)

add_compile_definitions(GLOG_USE_GLOG_EXPORT)

#set(BUILD_TYPE Release)
set(BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE ${BUILD_TYPE})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -O3 -s -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable ") #-Wno-unused-result



set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

add_compile_options(#-Werror#-Wpedantic #minor error
                   -Wreturn-type # warn no return 
                   -Wuninitialized # uninitlialized varibales
                  # -Wshadow # shadow local variable
                   -Werror=maybe-uninitialized
                   -Wmultistatement-macros)

option(arm_neon " arm_neon" ON)
option(build_test "build test" OFF)
if(${arm_neon})
add_definitions(-D__ARM_NEON__)

else()
add_definitions(-D__SSE2__)
endif()
add_definitions(-DOPENCV_ENABLE_MEMORY_SANITIZER)
set(EIGEN3_INCLUDE_DIR ${TOP_CMAKE_DIR}/third_party/eigen-3.4.0)
set(OpenCV_INCLUDE_DIRS ${TOP_CMAKE_DIR}/third_party/opencv-4.6.0/include/opencv4)
set(CERES_INCLUDE_DIRS ${TOP_CMAKE_DIR}/third_party/ceres-solver/include/)
set(CERES_LIBRARIES ${TOP_CMAKE_DIR}/third_party/ceres-solver/lib/libceres.a)
set(Opencv_lib_dir ${TOP_CMAKE_DIR}/third_party/opencv-4.6.0/lib/mr527)
set(yaml_lib_dir ${TOP_CMAKE_DIR}/third_party/yaml-cpp/include/)
set(yaml_lib ${TOP_CMAKE_DIR}/third_party/yaml-cpp/lib/libyaml-cpp.a)
# find_package(Eigen3 3.4.0 EXACT) #version 3.4.0
# lie group and lie algebra
# find_package(OpenCV REQUIRED) #version 4.6.0
set(OpenCV_LIBS
    ${Opencv_lib_dir}/libopencv_core.so            # ${OpenCV_LIBRARIES}
    ${Opencv_lib_dir}/libopencv_imgcodecs.so
    ${Opencv_lib_dir}/libopencv_imgproc.so
    ${Opencv_lib_dir}/libopencv_features2d.so
    ${Opencv_lib_dir}/libopencv_calib3d.so
    /home/lyp/project/mowerpack/mowerpack_all/ld_third_part/lib/libopencv_video.a
    # ${Opencv_lib_dir}/libopencv_video.so
)
set(SUTE_LIB_DIR "/home/lyp/project/mowerpack/mowerpack_all/third_party/clangarm64/lib")
set(SUTI_LIBS
 ${SUTE_LIB_DIR}/libamd.a  
 ${SUTE_LIB_DIR}/libcxsparse.a
#  ${SUTE_LIB_DIR}/libbtf.a 
#  ${SUTE_LIB_DIR}/libcamd.a
 ${SUTE_LIB_DIR}/libccolamd.a 
#  ${SUTE_LIB_DIR}/libcholmod.a
 ${SUTE_LIB_DIR}/libcolamd.a  
#  ${SUTE_LIB_DIR}/libcxsparse.a 
#  ${SUTE_LIB_DIR}/libklu.a      
#  ${SUTE_LIB_DIR}/libldl.a
#  ${SUTE_LIB_DIR}/librbio.a              
#  ${SUTE_LIB_DIR}/libspqr.a
#  ${SUTE_LIB_DIR}/libumfpack.a
#  ${TOP_CMAKE_DIR}/third_party/lapack/lib/liblapack.a
#  ${SUTE_LIB_DIR}/libsuitesparseconfig.so

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

get_directory_property(out_var INCLUDE_DIRECTORIES)
string(REPLACE  "eigen3" "eigen5" new_include_dirs ${out_var})
set_directory_properties(DIRECTORY  PROPERTY  jarvis INCLUDE_DIRECTORIES ${new_include_dirs})
get_directory_property(out_var INCLUDE_DIRECTORIES)
# message(******************8***${out_var})
##
INCLUDE_DIRECTORIES(
  # ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/jarvis
  ${PROJECT_SOURCE_DIR}/third_part/rtm/includes/
  ${yaml_lib_dir}/
  ${Boost_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
)


add_subdirectory("${PROJECT_SOURCE_DIR}/third_part/rtm")
##
file(GLOB_RECURSE ALL_LIBRARY_HDRS "jarvis/*.hpp" "jarvis/*.h" )
file(GLOB_RECURSE ALL_LIBRARY_SRCS "jarvis/*.cc" "jarvis/*.cpp" "slip_detection/*.cc")
file(GLOB_RECURSE NOT_USE_SRC "jarvis/camera_models/chessboard/*.cc")
##
file(GLOB_RECURSE OJBECT_LIBRARY_SRCS "jarvis/object*.cc")
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${OJBECT_LIBRARY_SRCS})
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${NOT_USE_SRC})
##
file(GLOB_RECURSE ALL_TEST_SRCS  "jarvis/*_test.cc" "jarvis/*_test.cpp")
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_TEST_SRCS})
# file(GLOB_RECURSE ALL_TEST_SRCS  "src/*_test.cc" "src/*_test.cpp")
add_library(${PROJECT_NAME} SHARED ${ALL_LIBRARY_SRCS}  )
target_link_libraries( ${PROJECT_NAME} PUBLIC ${PROTOBUF_INCLUDE_LIB}  )
target_include_directories( ${PROJECT_NAME}  SYSTEM PRIVATE   ${EIGEN3_INCLUDE_DIR})
target_link_libraries(
  ${PROJECT_NAME} PUBLIC
  glog
  # -lgfortran
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  ${Boost_LIBRARIES}
  ${SUTI_LIBS}
  ${yaml_lib}
  )

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




file(GLOB_RECURSE INSTALL_SOURCE_HDRS "jarvis/*.h" "slip_detection/*.h"
)

foreach(HDR ${INSTALL_SOURCE_HDRS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${HDR})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  install(
    FILES ${HDR}
    DESTINATION  include/${DIR}
  )
endforeach()
