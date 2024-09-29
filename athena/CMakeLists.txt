cmake_minimum_required(VERSION 3.14)
project(athena)

if (ENGINEER_OPTION STREQUAL "ON")
    message(STATUS "************ ATHENA ENGINEER MODE ************")
    set(ENGINEER_SUFFIX "_release_dd")
    set(PROJECT_NAME_ENGINEER ${PROJECT_NAME})

    add_definitions(-DENGINEER_OPTION)
    find_package_sense(ADFramework REQUIRED COMPONENTS ad_log)

    string(APPEND PROJECT_NAME_ENGINEER "${ENGINEER_SUFFIX}")
    add_library(${PROJECT_NAME_ENGINEER} SHARED
                ${CMAKE_CURRENT_SOURCE_DIR}/data_preprocess/occ_vector_generator.cpp
                ${CMAKE_CURRENT_SOURCE_DIR}/data_preprocess/map_data_preprocess.cpp
                ${CMAKE_CURRENT_SOURCE_DIR}/data_preprocess/agent_data_preprocess.cpp
                ${CMAKE_CURRENT_SOURCE_DIR}/data_preprocess/data_preprocess.cpp
    )
    target_include_directories(${PROJECT_NAME_ENGINEER} PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${ADFramework_INCLUDE_DIRS})
    ## Install headers and libs
    install(TARGETS ${PROJECT_NAME_ENGINEER} LIBRARY DESTINATION "lib")
else()
    message(STATUS "************ ATHENA DEVELOP MODE ************")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
    set(Python_INCLUDE_DIR "/Library/Frameworks/Python.framework/Versions/3.11/include/python3.11/")

    add_library(${PROJECT_NAME} STATIC
            ${CMAKE_CURRENT_SOURCE_DIR}/data_preprocess/occ_vector_generator.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/data_preprocess/map_data_preprocess.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/data_preprocess/agent_data_preprocess.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/data_preprocess/data_preprocess.cpp

    )
    target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

    include_directories(${Python_INCLUDE_DIR})
    find_package(pybind11 REQUIRED)
#
    pybind11_add_module(pybind_dlp
            ${CMAKE_CURRENT_SOURCE_DIR}/data_process_pybind/dlp_pybind.cpp)
    target_include_directories(pybind_dlp PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(pybind_dlp PRIVATE
            ${PROJECT_NAME}
    )
#    target_include_directories(${PROJECT_NAME} PUBLIC
#            ${CMAKE_CURRENT_SOURCE_DIR})
    add_executable(test_DLP
            ${CMAKE_CURRENT_SOURCE_DIR}/data_preprocess/main_test.cpp
    )
    target_include_directories(test_DLP PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
#    # find_package(CGAL REQUIRED)
    target_link_libraries(test_DLP ${PROJECT_NAME} ${PROJECT_NAME})
endif ()