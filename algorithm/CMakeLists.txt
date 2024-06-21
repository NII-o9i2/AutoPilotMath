cmake_minimum_required(VERSION 3.14)
project(algorithm)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++14")

set(EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/eigen)


add_subdirectory(math_common)
# add_subdirectory(idm)
add_subdirectory(env_simulator)
add_subdirectory(cilqr)
add_subdirectory(dcp_tree)
add_subdirectory(interaction_search)