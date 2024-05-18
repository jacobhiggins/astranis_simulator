cmake_minimum_required(VERSION 3.0)

# set project name
project(astranis_simulator)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# find required packages
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 REQUIRED)

# include directories
include_directories(include/astranis_simulator)

add_subdirectory(src)