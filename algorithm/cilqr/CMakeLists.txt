cmake_minimum_required(VERSION 3.14)
project(ilqr_math)

if (ENGINEER_OPTION STREQUAL "ON")
    message(STATUS "************ ILQR ENGINEER MODE ************")
    set(ENGINEER_SUFFIX "_release")
    set(PROJECT_NAME_ENGINEER ${PROJECT_NAME})
     
    add_definitions(-DENGINEER_OPTION)
    # Eigen3
    set(EIGEN_ROOT "${PATH_SOURCE_3RDPARTY}/include/eigen3/")
    find_package_sense(ADFramework REQUIRED COMPONENTS ad_log)

    include_directories(${EIGEN_ROOT})
    message(EIGEN_ROOT ${EIGEN_ROOT})

    string(APPEND PROJECT_NAME_ENGINEER "${ENGINEER_SUFFIX}")
    add_library(${PROJECT_NAME_ENGINEER} SHARED
            ${CMAKE_CURRENT_SOURCE_DIR}/core/ilqr_lon_condition.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/core/ilqr_tree_solver.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/core/ilqr_tree_interface.cpp
    )
    target_link_libraries(${PROJECT_NAME_ENGINEER}
            math_common
    )
    target_include_directories(${PROJECT_NAME_ENGINEER} PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${ADFramework_INCLUDE_DIRS})
            ## Install headers and libs
    install(TARGETS ${PROJECT_NAME_ENGINEER} LIBRARY DESTINATION "lib")
else()
    message(STATUS "************ ILQR DEVELOP MODE ************")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
    set(Python_INCLUDE_DIR "/Library/Frameworks/Python.framework/Versions/3.11/include/python3.11/")
    include_directories(${EIGEN3_INCLUDE_DIR})
    message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")
    add_library(${PROJECT_NAME} STATIC
            ${CMAKE_CURRENT_SOURCE_DIR}/core/ilqr_lon_condition.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/core/ilqr_tree_solver.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/core/ilqr_tree_interface.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/ilqr_tree_develop_interface.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/lateral_longitudinal_ilqr.cpp
    )
    target_link_libraries(${PROJECT_NAME}
            math_common
            env_simulator
    )

    include_directories(${Python_INCLUDE_DIR})
    find_package(pybind11 REQUIRED)

    pybind11_add_module(pybind_ilqr
            ${CMAKE_CURRENT_SOURCE_DIR}/ilqr_pybind.cpp)
    target_link_libraries(pybind_ilqr PRIVATE
            math_common
            ${PROJECT_NAME}
    )
    target_include_directories(${PROJECT_NAME} PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR})
    add_executable(test_ilqr
            ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    )
    # find_package(CGAL REQUIRED)
    target_link_libraries(test_ilqr math_common ${PROJECT_NAME})
endif ()