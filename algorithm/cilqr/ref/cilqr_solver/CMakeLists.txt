cmake_minimum_required(VERSION 3.14)
project(cilqr_solver VERSION 1.0.0 LANGUAGES CXX)
message(STATUS "${Green}[${PROJECT_NAME}] Building....${ColourReset}")

set(3RDPARTY_STATIC_DEPS
    cereal
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

# Add proto_type lib
add_library(${PROJECT_NAME} SHARED)
file(GLOB PROTO_SRC ${CMAKE_CURRENT_BINARY_DIR}/../auto_generate/proto/*.pb.cc ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
target_sources(${PROJECT_NAME} PRIVATE ${PROTO_SRC})

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    PUBLIC
        ${PATH_SOURCE_3RDPARTY}/include
        ${EIGEN_ROOT}
        ${common_proto_includes}
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        ${3rdparty_libraries}
)

add_library(${export_name}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

# Install lib and headers
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION "lib")

## Unittests
include(CTest)
add_subdirectory(unittests)