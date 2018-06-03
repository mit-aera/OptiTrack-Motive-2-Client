CMAKE_MINIMUM_REQUIRED(VERSION 3.3.0)
SET(CMAKE_COLOR_MAKEFILE ON)
SET(CMAKE_VERBOSE_MAKEFILE OFF)

PROJECT(OptitrackClient)

# Use C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-unused-function")

# Add source executables and libraries
add_library(motionCaptureClientFramework SHARED motionCaptureClientFramework.cpp)
add_executable (motionCaptureReceiver motionCaptureReceiver.cpp)

# Add external dependencies
find_package (Eigen3 REQUIRED NO_MODULE)
find_package(Boost COMPONENTS program_options REQUIRED)

INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

# Link source library with Boost.
target_link_libraries(motionCaptureReceiver motionCaptureClientFramework ${Boost_LIBRARIES})