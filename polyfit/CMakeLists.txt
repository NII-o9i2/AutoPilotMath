cmake_minimum_required(VERSION 3.14)
project(poly_fit)

find_package(osqp REQUIRED)
add_compile_options("-fPIC")

set(SPLINE_SMOOTH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/spline_smoothing)
file(GLOB_RECURSE SPLINE_SMOOTH_SOURCES ${SPLINE_SMOOTH_DIR}/*.cpp)

set(SPLINE_SMOOTH_NEW_DIR ${CMAKE_CURRENT_SOURCE_DIR}/spline_new)
file(GLOB_RECURSE SPLINE_SMOOTH_NEW_SOURCES ${SPLINE_SMOOTH_NEW_DIR}/*.cpp)

add_library(${PROJECT_NAME} STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/polyfit.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/polyfit_osqp.cpp
        ${SPLINE_SMOOTH_SOURCES}
        ${SPLINE_SMOOTH_NEW_SOURCES}
        )
target_link_libraries(${PROJECT_NAME} PUBLIC
        osqp::osqp)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
