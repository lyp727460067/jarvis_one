project(vslam)

# If we will compile with aruco support
option(ENABLE_ARUCO_TAGS "Enable or disable aruco tag (disable if no contrib modules)" OFF)
if (NOT ENABLE_ARUCO_TAGS)
    add_definitions(-DENABLE_ARUCO_TAGS=0)
    message(WARNING "DISABLING ARUCOTAG TRACKING!")
else ()
    add_definitions(-DENABLE_ARUCO_TAGS=1)
endif ()

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fopenmp -lpthread -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops -Wno-error")

# Include our header files
include_directories(
        src
        ${TOP_CMAKE_DIR}/libs/eigen3                                  # ${EIGEN3_INCLUDE_DIR}
        ${TOP_CMAKE_DIR}/libs/opencv-4.6.0/include/opencv4     # ${OpenCV_INCLUDE_DIR}
        ${TOP_CMAKE_DIR}/libs/mpsdk
        ${TOP_CMAKE_DIR}/libs/libmpc/include
)

# Set link libraries used by all binaries
link_directories(
    ${TOP_CMAKE_DIR}/third_party/opencv-4.6.0/lib/${PLATFORM_NAME}
)
list(APPEND THIRD_PARTY_LIBS
        libopencv_core.so            # ${OpenCV_LIBRARIES}
        libopencv_imgcodecs.so
        libopencv_imgproc.so
        libopencv_features2d.so
        libopencv_calib3d.so
        libopencv_dnn.so
        libopencv_flann.so
        libopencv_video.so
)
if(ENABLE_ARUCO_TAGS)
    list(APPEND THIRD_PARTY_LIBS
        libopencv_aruco.so
        libopencv_tracking.so
    )
endif ()

file(GLOB_RECURSE SRC_LIST
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

ADD_EXECUTABLE(slam_capture ${SRC_LIST})
TARGET_LINK_LIBRARIES(slam_capture ${THIRD_PARTY_LIBS} mpc)

set(PROJECTNAME "slam_capture")
IF (UNIX)
    if(BUILD_DEBUG)
      ADD_CUSTOM_COMMAND(TARGET ${PROJECTNAME}
              POST_BUILD
              # 原始文件保存为debug后缀文件
              COMMAND cp ${EXECUTABLE_OUTPUT_PATH}/${PROJECTNAME} ${EXECUTABLE_OUTPUT_PATH}/${PROJECTNAME}.debug
              # 压缩exe文件体积
              COMMAND ${CMAKE_STRIP} ${EXECUTABLE_OUTPUT_PATH}/${PROJECTNAME}
      )
    endif(BUILD_DEBUG)
ENDIF (UNIX)
