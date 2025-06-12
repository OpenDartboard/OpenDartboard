cmake_minimum_required(VERSION 3.17)
project(opendartboard LANGUAGES CXX)

# Tell CMake where the static ncnn & OpenCV we just built/installed live.
set(CMAKE_PREFIX_PATH "/usr/local" ${CMAKE_PREFIX_PATH})

# Find threads package for threading support
find_package(Threads REQUIRED)

# Use pkg-config to find OpenCV if standard find_package fails
find_package(OpenCV QUIET)
if(NOT OpenCV_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(OPENCV REQUIRED opencv4)
    set(OpenCV_INCLUDE_DIRS ${OPENCV_INCLUDE_DIRS})
    set(OpenCV_LIBS ${OPENCV_LIBRARIES})
endif()

# Find ncnn package
find_package(ncnn QUIET)
if(NOT ncnn_FOUND)
    message(WARNING "ncnn not found in system, using library in /usr/local")
    set(NCNN_INCLUDE_DIRS "/usr/local/include/ncnn")
    set(NCNN_LIBRARIES "/usr/local/lib/libncnn.a")
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GSTREAMER_BASE REQUIRED gstreamer-base-1.0)
pkg_check_modules(GSTREAMER_VIDEO REQUIRED gstreamer-video-1.0)

add_executable(opendartboard
    src/main.cpp
    src/scorer/scorer.cpp
    # src/detector/ai/yolo_detector.cpp
    src/detector/geometry/geometry_detector.cpp
    src/detector/geometry/geometry_dartboard.cpp
    src/detector/geometry/geometry_calibration.cpp 
)

target_include_directories(opendartboard PRIVATE
    /usr/local/include
    ${OpenCV_INCLUDE_DIRS}
    ${NCNN_INCLUDE_DIRS}
    ${GSTREAMER_INCLUDE_DIRS}
    ${GSTREAMER_APP_INCLUDE_DIRS}
    ${GSTREAMER_BASE_INCLUDE_DIRS}
    ${GSTREAMER_VIDEO_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    /usr/include/opencv4  # Add explicit path to OpenCV headers
)

target_link_libraries(opendartboard
    PRIVATE
        ${OpenCV_LIBS}
        ${NCNN_LIBRARIES}
        ${GSTREAMER_LIBRARIES}
        ${GSTREAMER_APP_LIBRARIES}
        ${GSTREAMER_BASE_LIBRARIES}
        ${GSTREAMER_VIDEO_LIBRARIES}
        Threads::Threads  # Add pthread linking
)

target_compile_features(opendartboard PRIVATE cxx_std_17)