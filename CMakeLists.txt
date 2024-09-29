cmake_minimum_required(VERSION 3.14)
project(autopilot_math)

option(ENGINEER_OPTION OFF)

if(${ENGINEER_OPTION})
        message(STATUS "************ AUTOPILOT MATH ENGINEER MODE ************ ")

        # set(SDK_ROOT_PATH "/home/sensetime/ws/rscl_aarch64/senseauto-pilot-decision/pilot_decision_sdk")
        set(EIGEN3_INCLUDE_DIR ${SDK_ROOT_PATH}/deps/Eigen/eigen3)
        include_directories(${EIGEN3_INCLUDE_DIR})
        add_subdirectory(algorithm/math_common)
        add_subdirectory(algorithm/cilqr)
        add_subdirectory(algorithm/dcp_tree)
        add_subdirectory(algorithm/interaction_search)
        add_subdirectory(athena)
else()
        message(STATUS "************ AUTOPILOT MATH DEVELOP MODE ************ ")
        set(EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/eigen)
        include_directories(${EIGEN3_INCLUDE_DIR})
        include_directories(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)

        set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
        option(BENCHMARK_DOWNLOAD_DEPENDENCIES ON)
        option(BENCHMARK_ENABLE_GTEST_TESTS OFF)
        option(BENCHMARK_USE_BUNDLED_GTEST OFF)

        add_definitions(-DBENCHMARK_MODE)

        include_directories(${CMAKE_CURRENT_SOURCE_DIR}/polyfit)
        include_directories(polyfit)
        add_subdirectory(polyfit)
        add_subdirectory(algorithm/math_common)
        add_subdirectory(algorithm/env_simulator)
        add_subdirectory(algorithm/dcp_tree)
        add_subdirectory(algorithm/cilqr)
        add_subdirectory(thirdparty/benchmark)
        add_subdirectory(thirdparty/pybind)
        add_subdirectory(athena)

        # add_subdirectory(njson)
        add_subdirectory(jupyter_notebook)

        add_executable(${PROJECT_NAME}
                bench_test.cpp
        )
        target_link_libraries(${PROJECT_NAME} PUBLIC
                benchmark::benchmark
                poly_fit
                math_common
                ilqr_sim

                # njson_test
        )
endif()