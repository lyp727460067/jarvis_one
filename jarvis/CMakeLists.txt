cmake_minimum_required(VERSION 3.0)
project(jarvis VERSION 1.0.0)



set(BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE ${BUILD_TYPE})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse2 -msse4.1 -fPIC -Ofast  -mpopcnt") #-Wno-unused-result


# #set output folders
# set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
# message(STATUS ${CMAKE_BINARY_DIR})
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)

add_compile_options(#-Werror#-Wpedantic #minor error
                   -Wreturn-type # warn no return 
                   -Wuninitialized # uninitlialized varibales
                  # -Wshadow # shadow local variable
                   -Wmultistatement-macros)


option(build_test "build test" ON)
find_package(Eigen3 3.4.0 EXACT) #version 3.4.0
# lie group and lie algebra
find_package(OpenCV REQUIRED) #version 4.6.0
# ceres optimization replace gtsam later
find_package(Ceres REQUIRED)
find_package(Glog REQUIRED) #version 0.6.0
# test system
find_package(GTest  REQUIRED) #version 1.12.0
#
find_package(yaml-cpp  REQUIRED) #version 0.7.0
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
)

##
file(GLOB_RECURSE ALL_LIBRARY_HDRS "jarvis/*.hpp" "jarvis/*.h" )
file(GLOB_RECURSE ALL_LIBRARY_SRCS "jarvis/*.cc" "jarvis/*.cpp")
##
#file(GLOB_RECURSE VINS_LIBRARY_SRCS "src/mapping/tracking/*test_ros.cc" "src/mapping/tracking/*test_ros.cpp"
# "src/mapping/tracking/*test_ros.h")
#list(REMOVE_ITEM ALL_LIBRARY_SRCS ${VINS_LIBRARY_SRCS})
##
# file(GLOB_RECURSE ALL_TEST_SRCS  "src/*_test.cc" "src/*_test.cpp")
add_library(${PROJECT_NAME} SHARED ${ALL_LIBRARY_SRCS}  ${ALL_LIBRARY_HDRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${PROTOBUF_LIBRARY})
target_link_libraries(${PROJECT_NAME} PUBLIC ${GRPC_LIB})
target_link_libraries( ${PROJECT_NAME} PUBLIC ${PROTOBUF_INCLUDE_LIB}  )
target_link_libraries(
  ${PROJECT_NAME} PUBLIC
  glog::glog
  yaml-cpp
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  ${Sophus_LIBRARIES}
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