cmake_minimum_required(VERSION 3.5.0)
project(orb_slam2_core)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-O3")

find_package(catkin REQUIRED COMPONENTS
        g2o dbow2
        )


find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Pangolin REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES orb_slam2_core
        CATKIN_DEPENDS g2o dbow2
)
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PROJECT_SOURCE_DIR}/include
        ${EIGEN3_INCLUDE_DIR}
        ${Pangolin_INCLUDE_DIRS}
)

aux_source_directory(src orb2_SRCS)

add_library(${PROJECT_NAME} SHARED ${orb2_SRCS})
target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${catkin_LIBRARIES}
        )

message(STATUS "Unzip Vocabulary Document...")
execute_process(COMMAND tar Jxf ${CMAKE_CURRENT_SOURCE_DIR}/vocabulary/ORBvoc.tar.xz)