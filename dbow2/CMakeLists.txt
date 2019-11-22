cmake_minimum_required(VERSION 3.5.0)
project(dbow2)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-O3")
# Compiler specific options for gcc
#SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -march=native")
#SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -march=native")

find_package(catkin REQUIRED)
find_package(OpenCV 3 REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES dbow2
)

include_directories(include)

aux_source_directory(src DBOW2_SRCS)

include_directories(${OpenCV_INCLUDE_DIRS})
add_library(dbow2 SHARED ${DBOW2_SRCS})
target_link_libraries(dbow2 ${OpenCV_LIBS})