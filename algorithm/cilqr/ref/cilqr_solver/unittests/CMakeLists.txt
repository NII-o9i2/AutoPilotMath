##
# Copyright (C) 2023 by SenseTime Group Limited. All rights reserved.
# Su Chenhui <suchenhui@sensetime.com>
##

find_package(Threads REQUIRED)
set(INSTALL_TEST OFF)
set(3RDPARTY_STATIC_DEPS
    gtest
    gflags
    glog
    cereal
    json11
)

set(3RDPARTY_SHARED_DEPS
    protobuf
)

find_3rdparty_package(
    HAS_ROOT_INCLUDE
    HEADER 3rdparty_includes
    LIBRARIES 3rdparty_libraries
    STATIC_DEPS 3RDPARTY_STATIC_DEPS
    SHARED_DEPS 3RDPARTY_SHARED_DEPS
)

file(GLOB unittests ${CMAKE_CURRENT_LIST_DIR}/*.cpp)

foreach(unittest ${unittests})
    string(REGEX MATCH "[^/]+$" SRC_FILE ${unittest})
    string(REPLACE ".cpp" "" APP_NAME ${SRC_FILE})
    if(TARGET ${APP_NAME})
        continue()
    endif()
    get_filename_component(unittest_name ${unittest} NAME)
    add_executable(${APP_NAME} ${unittest})
    target_link_directories(${APP_NAME}
        PRIVATE
    )

    target_include_directories(${APP_NAME}
        PRIVATE
            ${CMAKE_SOURCE_DIR}/${PROJECT_NAME}/include
            ${3rdparty_includes}
    )
    target_link_libraries(${APP_NAME}
        PRIVATE
            ${3rdparty_libraries}
            ${PROJECT_NAME}
            -lpthread -lm
    )
    add_test(NAME ${APP_NAME} COMMAND ${CMAKE_CURRENT_BINARY_DIR}/${APP_NAME})
endforeach()
