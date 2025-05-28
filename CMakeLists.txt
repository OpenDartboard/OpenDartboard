cmake_minimum_required(VERSION 3.17)
project(opendartboard LANGUAGES CXX)

# Tell CMake where the static ncnn & OpenCV we just built/installed live.
set(CMAKE_PREFIX_PATH "/usr/local" ${CMAKE_PREFIX_PATH})

find_package(OpenCV REQUIRED)
# find_library(NCNN_LIB ncnn PATHS /usr/local/lib)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstapp-1.0)
pkg_check_modules(GSTREAMER_BASE REQUIRED gstbase-1.0)
pkg_check_modules(GSTREAMER_VIDEO REQUIRED  gstvideo-1.0)

add_executable(opendartboard
    src/main.cpp
    src/scorer.cpp
)

target_include_directories(opendartboard PRIVATE /usr/local/include)
target_link_libraries(opendartboard
    PRIVATE
        # ${NCNN_LIB}
        ${OpenCV_LIBS}
        gstreamer-1.0
        gstapp-1.0
        gstbase-1.0
        gstvideo-1.0
)

target_compile_features(opendartboard PRIVATE cxx_std_17)