cmake_minimum_required(VERSION 3.14)
project(bench_test)

#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-std=c++14")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")

option(BENCHMARK_DOWNLOAD_DEPENDENCIES ON)
option(BENCHMARK_ENABLE_GTEST_TESTS OFF)
option(BENCHMARK_USE_BUNDLED_GTEST OFF)

add_definitions(-DBENCHMARK_MODE)

set(EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/eigen)
include_directories(${EIGEN3_INCLUDE_DIR})


include_directories(polyfit)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/polyfit)

add_subdirectory(polyfit)
add_subdirectory(algorithm/math_common)
add_subdirectory(algorithm/env_simulator)
add_subdirectory(algorithm/cilqr)
add_subdirectory(thirdparty/benchmark)
add_subdirectory(thirdparty/pybind)
#add_subdirectory(njson)
add_subdirectory(jupyter_notebook)


add_executable(${PROJECT_NAME}
        bench_test.cpp
        )

target_link_libraries(${PROJECT_NAME} PUBLIC
        benchmark::benchmark
        poly_fit
        math_common
        ilqr_sim
        #njson_test
        )