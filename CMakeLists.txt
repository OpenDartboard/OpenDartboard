cmake_minimum_required(VERSION 3.17)
project(opendartboard LANGUAGES CXX)

# Add FetchContent module for dependency management
include(FetchContent)

# Fetch nlohmann/json
FetchContent_Declare(
    nlohmann_json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.12.0
)

# Fetch cpp-httplib (MIT licensed)
FetchContent_Declare(
    httplib
    GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
    GIT_TAG v0.14.3
)



# Make dependencies available
FetchContent_MakeAvailable(nlohmann_json httplib)

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

find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GSTREAMER_BASE REQUIRED gstreamer-base-1.0)
pkg_check_modules(GSTREAMER_VIDEO REQUIRED gstreamer-video-1.0)

add_executable(opendartboard
    src/main.cpp
    src/scorer/scorer.cpp
    # Communication components
    src/communication/score_queue.cpp
    src/communication/websocket_service.cpp
    # src/detector/ai/yolo_detector.cpp // Uncomment when AI detector is ready
    src/detector/geometry/geometry_detector.cpp
    # calibration files
    src/detector/geometry/calibration/geometry_calibration.cpp
    src/detector/geometry/calibration/color_processing.cpp
    src/detector/geometry/calibration/contour_processing.cpp
    src/detector/geometry/calibration/roi_processing.cpp
    src/detector/geometry/calibration/ellipse_processing.cpp
    src/detector/geometry/calibration/bull_processing.cpp
    src/detector/geometry/calibration/mask_processing.cpp
    src/detector/geometry/calibration/wire_processing.cpp
    src/detector/geometry/calibration/perspective_processing.cpp
    src/detector/geometry/calibration/orientation_processing.cpp
    # detection files
    # src/detector/geometry/detection/geometry_detection.cpp
    src/detector/geometry/detection/motion_processing.cpp
    src/detector/geometry/detection/dart_processing.cpp
    src/detector/geometry/detection/score_processing.cpp
)

# Add include directories for easier includes
target_include_directories(opendartboard PRIVATE
    /usr/local/include
    ${OpenCV_INCLUDE_DIRS}
    ${NCNN_INCLUDE_DIRS}
    ${GSTREAMER_INCLUDE_DIRS}
    ${GSTREAMER_APP_INCLUDE_DIRS}
    ${GSTREAMER_BASE_INCLUDE_DIRS}
    ${GSTREAMER_VIDEO_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/utils
    /usr/include/opencv4  # Add explicit path to OpenCV headers
    ${httplib_SOURCE_DIR}  # Add httplib include path (not libwebsockets)
)

target_link_libraries(opendartboard
    PRIVATE
        ${OpenCV_LIBS}
        ${NCNN_LIBRARIES}
        ${GSTREAMER_LIBRARIES}
        ${GSTREAMER_APP_LIBRARIES}
        ${GSTREAMER_BASE_LIBRARIES}
        ${GSTREAMER_VIDEO_LIBRARIES}
        nlohmann_json::nlohmann_json
        ${CMAKE_DL_LIBS}
        Threads::Threads  # Add pthread linking
)

# Define version for the project
target_compile_definitions(opendartboard
  PRIVATE
    APP_VERSION="${APP_VERSION}"
)

target_compile_features(opendartboard PRIVATE cxx_std_17)