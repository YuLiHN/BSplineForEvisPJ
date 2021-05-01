cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(bspline)

set( CMAKE_CXX_FLAGS "-std=c++11 -O2" )

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(Sophus REQUIRED)
include_directories( ${Sophus_INCLUDE_DIRS} )

#dependencies#
include_directories(${PROJECT_SOURCE_DIR}/include)
add_subdirectory( src )
add_subdirectory( test )




