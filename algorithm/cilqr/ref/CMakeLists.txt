cmake_minimum_required(VERSION 3.14.0)
project(cilqr_solver)

# CXX version
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED on)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ADBuildTools REQUIRED)

set(PATH_SOURCE_3RDPARTY ${SENSEAUTO_DEV_INSTALLED_ROOT}/opt/senseauto/senseauto-3rdparty/active/3rdparty)
# Eigen3
set(EIGEN_ROOT "${PATH_SOURCE_3RDPARTY}/include/eigen3/")

include(${BUILDTOOLS_UTILS_CMAKE_DIR}/SenseIdlUtils.cmake)
include(${BUILDTOOLS_UTILS_CMAKE_DIR}/SenseInstallUtils.cmake)

## handle protobuf
if (CMAKE_CROSSCOMPILING)
    set(PROTOC_ENV LD_LIBRARY_PATH=${PATH_SOURCE_3RDPARTY}/bin/protoc/linux_x86_64/)
    set(PROTOC_BIN ${PATH_SOURCE_3RDPARTY}/bin/protoc/linux_x86_64/protoc)
else()
    set(PROTOC_ENV LD_LIBRARY_PATH=${PATH_SOURCE_3RDPARTY}/bin/protoc/linux_${PLATFORM_NAME}/)
    set(PROTOC_BIN ${PATH_SOURCE_3RDPARTY}/bin/protoc/linux_${PLATFORM_NAME}/protoc)
endif()
# to prevent recompile after cmake
execute_process(
    COMMAND
        mkdir -p auto_generate/proto_tmp
    COMMAND
        mkdir -p auto_generate/proto
    COMMAND
        mkdir -p ${CMAKE_CURRENT_SOURCE_DIR}/scripts/proto
    WORKING_DIRECTORY
        ${CMAKE_CURRENT_BINARY_DIR}
)

file(GLOB_RECURSE _proto_files "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto")
foreach (each_proto ${_proto_files})
    execute_process(
        COMMAND
            ${CMAKE_COMMAND} -E env ${PROTOC_ENV}
            ${PROTOC_BIN} --experimental_allow_proto3_optional -I=${CMAKE_CURRENT_SOURCE_DIR}/proto --cpp_out=${CMAKE_CURRENT_BINARY_DIR}/auto_generate/proto_tmp ${each_proto}
        WORKING_DIRECTORY
            ${CMAKE_CURRENT_BINARY_DIR}
        RESULT_VARIABLE ret
    )
    if (ret EQUAL "1")
        message(FATAL_ERROR "${BoldRed}failed to compile proto to cpp file${ColourReset}")
    endif()
    execute_process(
        COMMAND
            ${CMAKE_COMMAND} -E env ${PROTOC_ENV}
            ${PROTOC_BIN} --experimental_allow_proto3_optional -I=${CMAKE_CURRENT_SOURCE_DIR}/proto --python_out=${CMAKE_CURRENT_SOURCE_DIR}/scripts/proto ${each_proto}
        WORKING_DIRECTORY
            ${CMAKE_CURRENT_BINARY_DIR}
        RESULT_VARIABLE ret
    )
    if (ret EQUAL "1")
        message(FATAL_ERROR "${BoldRed}failed to compile proto to python file${ColourReset}")
    endif()
endforeach()

file(GLOB_RECURSE _proto_pb "${CMAKE_CURRENT_BINARY_DIR}/auto_generate/proto_tmp/*.pb.*[cc|h]")
foreach (each_proto_pb ${_proto_pb})
    string(REGEX REPLACE "${CMAKE_CURRENT_BINARY_DIR}/auto_generate/proto_tmp/" "" target_name ${each_proto_pb})
    execute_process(
        COMMAND
            ${CMAKE_COMMAND} -E copy_if_different ${each_proto_pb} ${CMAKE_CURRENT_BINARY_DIR}/auto_generate/proto/${target_name}
    )
endforeach()

execute_process(
    COMMAND
        rm -rf auto_generate/proto_tmp
    WORKING_DIRECTORY
        ${CMAKE_CURRENT_BINARY_DIR}
)
set(common_proto_includes ${CMAKE_CURRENT_BINARY_DIR}/auto_generate/)
include(CTest)

# build module
add_subdirectory(cilqr_solver)
