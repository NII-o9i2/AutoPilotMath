cmake_minimum_required(VERSION 3.14)
project(algorithm)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++14")

add_subdirectory(math_common)
add_subdirectory(dcp_tree)
add_subdirectory(idm)
add_subdirectory(env_simulator)
add_subdirectory(cilqr)