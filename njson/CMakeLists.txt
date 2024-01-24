cmake_minimum_required(VERSION 3.14)
project(njson_test)

find_package(osqp REQUIRED)

add_library(${PROJECT_NAME} STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/njson_test.cpp
        )
target_link_libraries(${PROJECT_NAME} PUBLIC
        osqp::osqp)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
