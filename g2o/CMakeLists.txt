cmake_minimum_required(VERSION 3.5.0)
project(g2o)

set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-O3")

SET(g2o_C_FLAGS)
SET(g2o_CXX_FLAGS)

find_package(catkin REQUIRED)

FIND_PACKAGE(OpenMP)
SET(G2O_USE_OPENMP OFF CACHE BOOL "Build g2o with OpenMP support (EXPERIMENTAL)")
IF (OPENMP_FOUND AND G2O_USE_OPENMP)
    SET(G2O_OPENMP 1)
    SET(g2o_C_FLAGS "${g2o_C_FLAGS} ${OpenMP_C_FLAGS}")
    SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE ${OpenMP_CXX_FLAGS}")
    MESSAGE(STATUS "Compiling with OpenMP support")
ENDIF (OPENMP_FOUND AND G2O_USE_OPENMP)

find_package(Eigen3 REQUIRED)

catkin_package(
        INCLUDE_DIRS src
        LIBRARIES g2o
)

include_directories(
        src
        ${EIGEN3_INCLUDE_DIR}
)

aux_source_directory(src/g2o/core G2O_CORE_SRCS)
aux_source_directory(src/g2o/stuff G2O_STUFF_SRCS)
aux_source_directory(src/g2o/types G2O_TYPES_SRCS)

ADD_LIBRARY(g2o SHARED
        ${G2O_CORE_SRCS}
        ${G2O_STUFF_SRCS}
        ${G2O_TYPES_SRCS})