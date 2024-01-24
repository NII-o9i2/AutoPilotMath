cmake_minimum_required(VERSION 3.5)
project(idm_pybind)

# 设置 Python 头文件的路径
# set(Python_INCLUDE_DIR "/Library/Frameworks/Python.framework/Versions/3.11/include/python3.11/")
# include_directories(${Python_INCLUDE_DIR})

# 添加 pybind11 子模块
find_package(pybind11 REQUIRED)

# 添加你的 C++ 源文件
pybind11_add_module(pybind_idm
        ${CMAKE_CURRENT_SOURCE_DIR}/pybind_idm.cpp)

add_library(idm SHARED idm.cpp)

# 链接你的库（如果有的话）
target_link_libraries(pybind_idm PUBLIC idm)


# 设置版本信息（如果有的话）
target_compile_definitions(pybind_idm PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})
