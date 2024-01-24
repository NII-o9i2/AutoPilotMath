cmake_minimum_required(VERSION 3.14)
project(ilqr_sim)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
find_package(Eigen3 REQUIRED)
set(Python_INCLUDE_DIR "/Library/Frameworks/Python.framework/Versions/3.11/include/python3.11/")

include_directories(${EIGEN3_INCLUDE_DIR})

add_library(${PROJECT_NAME} STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/ilqr_lon_condition.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ilqr_solver.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ilqr_tree_solver.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/lateral_longitudinal_ilqr.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/init_traj_generator.cpp
)
target_link_libraries(${PROJECT_NAME}
        math_common
        env_simulator
        Eigen3::Eigen
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