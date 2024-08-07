cmake_minimum_required(VERSION 3.0)
project(jarvis VERSION 1.0.0)
add_compile_definitions(GLOG_USE_GLOG_EXPORT)
# set(BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE ${BUILD_TYPE})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)


add_compile_options(#-Werror#-Wpedantic #minor error
                   -Wreturn-type # warn no return 
                   -Wuninitialized # uninitlialized varibales
                  # -Wshadow # shadow local variable
                   -Werror=maybe-uninitialized
                   -Wmultistatement-macros)
set(BUILD_TYPE Release)

if(PLATFORM_mr527)
add_definitions(-DOPENCV_ENABLE_MEMORY_SANITIZER)
add_definitions(-D__ARM_NEON__)
add_definitions(-D__ARM_PLATFORM__)
##
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -O3 -s -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable ") #-Wno-unused-result
option(build_test "build test" OFF)

set(EIGEN3_INCLUDE_DIR ${TOP_CMAKE_DIR}/third_party/eigen-3.4.0)
set(OpenCV_INCLUDE_DIRS ${TOP_CMAKE_DIR}/third_party/opencv-4.6.0/include/opencv4)
set(CERES_INCLUDE_DIRS ${TOP_CMAKE_DIR}/third_party/ceres-solver/include/)
set(CERES_LIBRARIES ${TOP_CMAKE_DIR}/third_party/ceres-solver/lib/libceres.a)
set(Opencv_lib_dir ${TOP_CMAKE_DIR}/third_party/opencv-4.6.0/lib/mr527)
set(YAML_INCLUDE_DIRS ${TOP_CMAKE_DIR}/third_party/yaml-cpp/include/)
set(yaml_lib ${TOP_CMAKE_DIR}/third_party/yaml-cpp/lib/libyaml-cpp.a)


set(OpenCV_LIBS
    ${Opencv_lib_dir}/libopencv_core.so            # ${OpenCV_LIBRARIES}
    ${Opencv_lib_dir}/libopencv_imgcodecs.so
    ${Opencv_lib_dir}/libopencv_imgproc.so
    ${Opencv_lib_dir}/libopencv_features2d.so
    ${Opencv_lib_dir}/libopencv_calib3d.so
    # /home/lyp/project/mowerpack/mowerpack_all/ld_third_part/lib/libopencv_video.a
    ${Opencv_lib_dir}/libopencv_video.so
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
        glog::glog
        STATIC
        IMPORTED
        GLOBAL
)
set_target_properties(
        glog::glog
        PROPERTIES IMPORTED_LOCATION
        ${glog_binary_dir}
)
add_library(
        yaml-cpp
        STATIC
        IMPORTED
        GLOBAL
)
set_target_properties(
        yaml-cpp
        PROPERTIES IMPORTED_LOCATION
        ${yaml_lib}
)
get_directory_property(out_var INCLUDE_DIRECTORIES)
string(REPLACE  "eigen3" "eigen5" new_include_dirs ${out_var})
set_directory_properties(DIRECTORY  PROPERTY  jarvis INCLUDE_DIRECTORIES ${new_include_dirs})
get_directory_property(out_var INCLUDE_DIRECTORIES)
message(${LIBRARY_OUTPUT_PATH})
add_custom_target(COPY_COFIG ALL
  VERBATIM 
  COMMAND ${CMAKE_COMMAND} -E make_directory ${LIBRARY_OUTPUT_PATH}/configuration/
  COMMAND ${CMAKE_COMMAND} -E copy_directory 
  ${PROJECT_SOURCE_DIR}/configuration/ ${PROJECT_BINARY_DIR}/configuration/
  COMMENT "Copy config ${PROJECT_SOURCE_DIR}/configuration/ to  ${LIBRARY_OUTPUT_PATH}/configuration/"
) 

INCLUDE_DIRECTORIES(
  ${TOP_CMAKE_DIR}/vslam/
)
else()

add_definitions(-D__SSE2__)
find_package(Eigen3 3.4.0 EXACT)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(Glog REQUIRED) 
find_package(yaml-cpp  REQUIRED) #version 0.7.0
find_package(GTest  REQUIRED) #version 1.12.0

endif()

INCLUDE_DIRECTORIES(
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/jarvis
  ${PROJECT_SOURCE_DIR}/rtm/includes/
  ${YAML_INCLUDE_DIRS}/
  ${Boost_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
   
)
##copy config

add_subdirectory("${PROJECT_SOURCE_DIR}/rtm")
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
add_library(${PROJECT_NAME} STATIC ${ALL_LIBRARY_SRCS}  )
# target_link_libraries(${PROJECT_NAME} PUBLIC ${PROTOBUF_INCLUDE_LIB}  )
target_include_directories( ${PROJECT_NAME}  SYSTEM PRIVATE   ${EIGEN3_INCLUDE_DIR})
target_include_directories( ${PROJECT_NAME}  PUBLIC           ${GLOG_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME} PUBLIC
  glog::glog
  yaml-cpp
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES})

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
