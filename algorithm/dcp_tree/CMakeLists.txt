cmake_minimum_required(VERSION 3.14)
project(dcp_tree)


add_library(${PROJECT_NAME} STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/track_simulator.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dcp_tree.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/dcp_tree_mgr.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/main_test.cpp
)
target_link_libraries(${PROJECT_NAME} math_common)

#add_executable(test_dcp
#        ${CMAKE_CURRENT_SOURCE_DIR}/main_test.cpp
#)