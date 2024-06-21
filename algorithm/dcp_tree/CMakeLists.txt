cmake_minimum_required(VERSION 3.14)
project(dcp_tree)

if (ENGINEER_OPTION STREQUAL "ON")
    message(STATUS "************ DCP TREE ENGINEER MODE ************")
    set(ENGINEER_SUFFIX "_release_dd")
    set(PROJECT_NAME_ENGINEER ${PROJECT_NAME})
     
    add_definitions(-DENGINEER_OPTION)
    find_package_sense(ADFramework REQUIRED COMPONENTS ad_log)

    string(APPEND PROJECT_NAME_ENGINEER "${ENGINEER_SUFFIX}")
    add_library(${PROJECT_NAME_ENGINEER} SHARED
            ${CMAKE_CURRENT_SOURCE_DIR}/core/track_simulator.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/core/dcp_tree.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/core/dcp_motion_tree.cpp
    )
    target_link_libraries(${PROJECT_NAME_ENGINEER}
            math_common_dd
			ilqr_math_release_dd
    )
    target_include_directories(${PROJECT_NAME_ENGINEER} PUBLIC
            ${CMAKE_CURRENT_SOURCE_DIR}
            ${ADFramework_INCLUDE_DIRS})
            ## Install headers and libs
    install(TARGETS ${PROJECT_NAME_ENGINEER} LIBRARY DESTINATION "lib")

else()
    message(STATUS "************ DCP TREE DEVELOP MODE ************")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
    set(Python_INCLUDE_DIR "/Library/Frameworks/Python.framework/Versions/3.11/include/python3.11/")
    include_directories(${EIGEN3_INCLUDE_DIR})
    message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")
    add_library(${PROJECT_NAME} STATIC
            ${CMAKE_CURRENT_SOURCE_DIR}/core/track_simulator.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/core/dcp_tree.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/core/dcp_motion_tree.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/dcp_motion_tree_dev.cpp
    )
	target_link_libraries(${PROJECT_NAME}
			math_common_dd
			env_simulator
			ilqr_math
	)
	include_directories(${Python_INCLUDE_DIR})
	find_package(pybind11 REQUIRED)

	pybind11_add_module(pybind_dcp_tree
			${CMAKE_CURRENT_SOURCE_DIR}/dcp_tree_pybind.cpp)
	target_link_libraries(pybind_dcp_tree PRIVATE
			math_common_dd
			${PROJECT_NAME}
	)
	target_include_directories(${PROJECT_NAME} PUBLIC
			${CMAKE_CURRENT_SOURCE_DIR})
	add_executable(test_dcp
			${CMAKE_CURRENT_SOURCE_DIR}/main_test.cpp
	)
	# find_package(CGAL REQUIRED)
	target_link_libraries(test_dcp math_common_dd ${PROJECT_NAME})
endif ()