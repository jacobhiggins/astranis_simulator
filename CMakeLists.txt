cmake_minimum_required(VERSION 3.0)

# set project name
project(astranis_simulator)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# find required packages
find_package(Boost REQUIRED COMPONENTS unit_test_framework)
find_package(Eigen3 REQUIRED)

# include directories
include_directories(
    ${EIGEN3_INCLUDE_DIR}
    include/astranis_simulator
)

# enable testing
enable_testing()

# add subdirectories
add_subdirectory(src)
add_subdirectory(tests)