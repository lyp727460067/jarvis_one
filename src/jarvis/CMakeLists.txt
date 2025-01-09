cmake_minimum_required(VERSION 3.0)
project(jarvis_main VERSION 1.0.0)
# define GLOG_EXPORT __attribute__((visibility("default")))
add_compile_definitions(GLOG_USE_GLOG_EXPORT)
# set(BUILD_TYPE Debug)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)


add_compile_options(#-Werror#-Wpedantic #minor error
                   -Wreturn-type # warn no return 
                   -Wuninitialized # uninitlialized varibales
                  # -Wshadow # shadow local variable
                   -Werror=maybe-uninitialized
                   -Wmultistatement-macros)
set(BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE ${BUILD_TYPE})



set(ARUCO_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/third_part/opencv/aruco/include/
${PROJECT_SOURCE_DIR}/video/include/
)
file(GLOB_RECURSE ARUCO_SRC "third_part/opencv/aruco/src/*.cpp"
)



if(PLATFORM_mr527)
  add_definitions(-DOPENCV_ENABLE_MEMORY_SANITIZER)
  add_definitions(-D__ARM_NEON__)
  add_definitions(-D__ARM_PLATFORM__)
  ##ofast编译会有问题，需要查看原因
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}   -O3 -s  -Wno-unused-function -Wno-unused-variable -Wno-unused-but-set-variable ") #-Wno-unused-result
  option(build_test "build test" OFF)
  set(PROTOBUF_PROTOC_EXECUTABLE  ${TOP_CMAKE_DIR}/third_party/protobuf/lib/protoc)
  set(PROTOBUF_INCLUDE_DIRS ${TOP_CMAKE_DIR}/third_party/protobuf/)
  set(PROTOBUF_LIBRARIES  ${TOP_CMAKE_DIR}/third_party/protobuf/lib/libprotobuf.a)
  set(EIGEN3_INCLUDE_DIR ${TOP_CMAKE_DIR}/third_party/eigen-3.4.0)
  set(OpenCV_INCLUDE_DIRS ${TOP_CMAKE_DIR}/third_party/opencv-4.6.0/include/opencv4)
  set(CERES_INCLUDE_DIRS ${TOP_CMAKE_DIR}/third_party/ceres-solver/include/)
  set(CERES_LIBRARIES ${TOP_CMAKE_DIR}/third_party/ceres-solver/lib/libceres_suitspace.a)
  set(Opencv_lib_dir ${TOP_CMAKE_DIR}/third_party/opencv-4.6.0/lib/mr527)
  set(YAML_INCLUDE_DIRS ${TOP_CMAKE_DIR}/third_party/yaml-cpp/include)
  set(yaml_lib ${TOP_CMAKE_DIR}/third_party/yaml-cpp/lib/libyaml-cpp.a)

  # message(${YAML_INCLUDE_DIRS})
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
  set(glog_binary_dir ${TOP_CMAKE_DIR}/third_party/glog/lib/libglog.so)
  add_library(glog::glog STATIC IMPORTED GLOBAL)
  set_target_properties( glog::glog PROPERTIES IMPORTED_LOCATION  ${glog_binary_dir})
  add_library(yaml-cpp STATIC IMPORTED GLOBAL)
  set_target_properties( yaml-cpp PROPERTIES IMPORTED_LOCATION ${yaml_lib})
  get_directory_property(out_var INCLUDE_DIRECTORIES)
  string(REPLACE  "eigen3" "eigen5" new_include_dirs ${out_var})
  set_directory_properties(DIRECTORY  PROPERTY  jarvis INCLUDE_DIRECTORIES ${new_include_dirs})
  get_directory_property(out_var INCLUDE_DIRECTORIES)
  add_custom_target(COPY_COFIG ALL
    VERBATIM 
    COMMAND ${CMAKE_COMMAND} -E make_directory ${LIBRARY_OUTPUT_PATH}/configuration/
    COMMAND ${CMAKE_COMMAND} -E copy_directory 
    ${PROJECT_SOURCE_DIR}/configuration/ ${LIBRARY_OUTPUT_PATH}/configuration/
    COMMENT "Copy config ${PROJECT_SOURCE_DIR}/configuration/ to  ${LIBRARY_OUTPUT_PATH}/configuration/"
  ) 
  INCLUDE_DIRECTORIES(
    ${TOP_CMAKE_DIR}/vslam/
    ${PROJECT_BINARY_DIR}/vslam/src/jarvis/
  )
else()
  file(GLOB_RECURSE PB_NOT_USE_SRC "/jarvis/mapping/dbow/proto/*.cc"  )
  file(GLOB_RECURSE VDEO_SRC "third_part/opencv/video/src/*.cpp")
  set(VEDIO_INCLUDE_DIR  ${PROJECT_SOURCE_DIR}/third_part/opencv/aruco/include/ )
  list(APPEND ARUCO_INCLUDE_DIR  ${VEDIO_INCLUDE_DIR})
  list(APPEND  ARUCO_SRC ${VDEO_SRC})
  set(GENERATE_PROTOBUF ON) 
  option(build_test "build test" ON)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -msse4.1 -fPIC -O3  -mpopcnt") #-Wno-unused-result
  add_definitions(-D__SSE2__)
  find_package(Eigen3 3.4.0 EXACT)
  # set(OpenCV_DIR "/home/lyp/third_part/opencv/build") 
  ## export PKG_CONFIG_PATH=/home/lyp/third_part/opencv/build/:$PKG_CONFIG_PATH
  find_package(OpenCV REQUIRED COMPONENTS highgui core imgproc calib3d  imgcodecs ) 
#   message( "Found OpenCV version: ${OpenCV_VERSION}")
# message( "OpenCV Include Dir: ${OpenCV_INCLUDE_DIRS}")
# message( "OpenCV Libraries: ${OpenCV_LIBS}")
  find_package(Ceres REQUIRED)
  find_package(Glog REQUIRED) 
  find_package(yaml-cpp  REQUIRED) #version 0.7.0
  find_package(GTest  REQUIRED) #version 1.12.0
  
find_package(Protobuf  REQUIRED) 
endif()

INCLUDE_DIRECTORIES(
  ${ARUCO_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/jarvis
  ${PROJECT_SOURCE_DIR}/third_part/rtm/includes/
  ${YAML_INCLUDE_DIRS}/
  ${Boost_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
)
##copy config

add_subdirectory("${PROJECT_SOURCE_DIR}/third_part/rtm")
##

file(GLOB_RECURSE ALL_LIBRARY_HDRS "jarvis/*.hpp" "jarvis/*.h" )
file(GLOB_RECURSE ALL_LIBRARY_SRCS "jarvis/*.cc" "jarvis/*.cpp" "slip_detection/*.cc")

file(GLOB_RECURSE NOT_USE_SRC "jarvis/camera_models/chessboard/*.cc"  )
file(GLOB_RECURSE GRID_MAP_SRC "jarvis/grid_map/*.cc")
##



##
list(REMOVE_ITEM ALL_LIBRARY_SRCS ${NOT_USE_SRC} ${PB_NOT_USE_SRC})
list(APPEND ALL_LIBRARY_SRCS ${ARUCO_SRC})
##
file(GLOB_RECURSE ALL_TEST_SRCS  "jarvis/*_test.cc" "jarvis/*_test.cpp")

list(REMOVE_ITEM ALL_LIBRARY_SRCS ${ALL_TEST_SRCS} ${GRID_MAP_SRC})
list(REMOVE_ITEM GRID_MAP_SRC  ${ALL_TEST_SRCS})
file(GLOB_RECURSE GRID_LIBRARY_SRC "jarvis/common/*.cc" "jarvis/transform/*.cc")
file(GLOB_RECURSE GRID_LIBRARY_HDRS "jarvis/common/*.h" "jarvis/transform/*.h")

message(${GRID_MAP_SRC})
add_library(jarvis SHARED ${GRID_LIBRARY_SRC} ${GRID_MAP_SRC})
target_include_directories(jarvis   SYSTEM PRIVATE   ${EIGEN3_INCLUDE_DIR})
target_include_directories(jarvis   PUBLIC   ${PROJECT_SOURCE_DIR}/)
target_include_directories(jarvis  PUBLIC           ${GLOG_INCLUDE_DIRS})
target_link_libraries (jarvis PUBLIC glog::glog)
# file(GLOB_RECURSE ALL_TEST_SRCS  "src/*_test.cc" "src/*_test.cpp")
file(GLOB_RECURSE ALL_PROTOS "jarvis/mapping/*.proto")
set(ALL_PROTO_SRCS)
set(ALL_PROTO_HDRS)
####
##
if(GENERATE_PROTOBUF)
foreach(ABS_FIL ${ALL_PROTOS})
  file(RELATIVE_PATH REL_FIL ${PROJECT_SOURCE_DIR} ${ABS_FIL})
  get_filename_component(DIR ${REL_FIL} DIRECTORY)
  get_filename_component(FIL_WE ${REL_FIL} NAME_WE)
  list(APPEND ALL_PROTO_SRCS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc")
  list(APPEND ALL_PROTO_HDRS "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h")
  add_custom_command(
    OUTPUT "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.cc"
           "${PROJECT_BINARY_DIR}/${DIR}/${FIL_WE}.pb.h"
    COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
    ARGS --cpp_out  ${PROJECT_BINARY_DIR} -I
      ${PROJECT_SOURCE_DIR}/  ${ABS_FIL}
    DEPENDS ${ABS_FIL}
    COMMENT " C++ protocol buffer compiler on ${ABS_FIL}"
    VERBATIM
)
endforeach()
endif()
set_source_files_properties(${ALL_PROTO_SRCS} ${ALL_PROTO_HDRS} PROPERTIES GENERATED TRUE)
list(APPEND ALL_LIBRARY_HDRS ${ALL_PROTO_HDRS})
list(APPEND ALL_LIBRARY_SRCS ${ALL_PROTO_SRCS})

add_library(${PROJECT_NAME} STATIC ${ALL_LIBRARY_SRCS}  )


# target_link_libraries(${PROJECT_NAME} PUBLIC ${PROTOBUF_INCLUDE_LIB}  )
target_include_directories( ${PROJECT_NAME}  SYSTEM PRIVATE   ${EIGEN3_INCLUDE_DIR})
target_include_directories( ${PROJECT_NAME}  PUBLIC           ${GLOG_INCLUDE_DIRS})
target_link_libraries (${PROJECT_NAME} PUBLIC glog::glog)
# message(${PROTOBUF_LIBRARIES})
target_include_directories(${PROJECT_NAME} PRIVATE ${PROTOBUF_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${PROTOBUF_LIBRARIES})
# link_directories(${OpenCV_DIR}/lib/)
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})

# target_link_libraries(${PROJECT_NAME} PUBLIC glog::glog)
# target_compile_definitions (${PROJECT_NAME} INTERFACE GLOG_EXPORT=__attribute__((visibility("default"))))
target_link_libraries(
  ${PROJECT_NAME} PUBLIC
  yaml-cpp
  ${CERES_LIBRARIES}
  
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
      jarvis
      ${PROJECT_NAME}
      ${OpenCV_LIBS}
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
