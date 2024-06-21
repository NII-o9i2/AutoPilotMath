cmake_minimum_required(VERSION 3.14)
project(math_common_dd)

add_library(${PROJECT_NAME} SHARED
        ${CMAKE_CURRENT_SOURCE_DIR}/utils.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/nn_tool.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/interval.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR})

set_target_properties(${PROJECT_NAME} PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

if (ENGINEER_OPTION STREQUAL "ON")
## Install headers and libs
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION "lib")
endif()