cmake_minimum_required(VERSION 3.14)
project(env_simulator)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
set(CMAKE_BUILD_TYPE Debug)
find_package(Eigen3 REQUIRED)
set(Python_INCLUDE_DIR "/Library/Frameworks/Python.framework/Versions/3.11/include/python3.11/")

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../cilqr)
add_library(${PROJECT_NAME} STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/spline.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/lane.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/obstacle.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/env_simulator.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/free_space.cpp
)
target_link_libraries(${PROJECT_NAME}
        math_common
        Eigen3::Eigen
)
target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR})

include_directories(${Python_INCLUDE_DIR})
find_package(pybind11 REQUIRED)

pybind11_add_module(pybind_env_simulator
        ${CMAKE_CURRENT_SOURCE_DIR}/pybind_env_simulator.cpp)
target_link_libraries(pybind_env_simulator PRIVATE
        math_common
        ${PROJECT_NAME}
)

add_executable(test_env_simulator
        ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)
# find_package(CGAL REQUIRED)
target_link_libraries(test_env_simulator math_common ${PROJECT_NAME})