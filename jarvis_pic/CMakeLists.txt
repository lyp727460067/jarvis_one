cmake_minimum_required(VERSION 3.0.2)
project(jarvis_ros)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++2a")
find_package(OpenCV REQUIRED) 
find_package(jarvis)
find_package(Glog REQUIRED)
include_directories(
${OpenCV_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_pic_main  jarvis_pic_main
)
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_pic_main
  glog::glog
  jarvis
)
