cmake_minimum_required(VERSION 3.14)
project(jupyter_pybind)
set(Python_INCLUDE_DIR "/Library/Frameworks/Python.framework/Versions/3.11/include/python3.11/")

include_directories(${Python_INCLUDE_DIR})

pybind11_add_module(pybind_test
        ${CMAKE_CURRENT_SOURCE_DIR}/pybind_polyfit.cpp)

target_link_libraries(pybind_test PUBLIC poly_fit)

# EXAMPLE_VERSION_INFO is defined by setup.py and passed into the C++ code as a
# define (VERSION_INFO) here.
target_compile_definitions(pybind_test
        PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})
