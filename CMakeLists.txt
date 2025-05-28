cmake_minimum_required(VERSION 3.17)
project(opendartboard LANGUAGES CXX)

# Tell CMake where the static ncnn & OpenCV we just built/installed live.
set(CMAKE_PREFIX_PATH "/usr/local" ${CMAKE_PREFIX_PATH})

find_package(OpenCV REQUIRED)
find_library(NCNN_LIB ncnn PATHS /usr/local/lib)

add_executable(opendartboard
    src/main.cpp
    src/scorer.cpp
)

target_include_directories(opendartboard PRIVATE /usr/local/include)
target_link_libraries(opendartboard
    PRIVATE
        ${NCNN_LIB}
        ${OpenCV_LIBS}
        Gst::app         # GStreamer app-sink wrapper
        Gst::base
        Gst::video
)

target_compile_features(opendartboard PRIVATE cxx_std_17)