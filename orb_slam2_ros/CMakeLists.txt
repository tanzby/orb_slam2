cmake_minimum_required(VERSION 3.5.0)
project(orb_slam2_ros)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "-O3")

find_package(catkin REQUIRED COMPONENTS
        orb_slam2_core roscpp tf sensor_msgs image_transport cv_bridge
        )

catkin_package(
        #    INCLUDE_DIRS include
        CATKIN_DEPENDS orb_slam2_core roscpp tf sensor_msgs image_transport cv_bridge
)
include_directories(
        ${catkin_INCLUDE_DIRS}
)


add_executable(stereo src/stereo.cc)
add_executable(mono src/mono.cc)
add_executable(rgbd src/rgbd.cc)

target_link_libraries(stereo ${catkin_LIBRARIES})
target_link_libraries(mono ${catkin_LIBRARIES})
target_link_libraries(rgbd ${catkin_LIBRARIES})
