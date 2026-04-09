cmake_minimum_required(VERSION 3.13)

project(Torpedo_EX60 C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(torpedo_main
    src/main.c
    src/common_Control/actuators.c
    src/common_Control/control.c
    src/common_Control/telemetry.c
    src/common_Control/transform.c
    src/common_Control/actuator_model.c
    src/common_Control/allocator.c
    src/common_Control/vehicle_config.c
    src/Debug/logger.c
)

target_include_directories(torpedo_main PRIVATE
    ${CMAKE_SOURCE_DIR}/lib/c_library_v2
    ${CMAKE_SOURCE_DIR}/src
)

target_compile_options(torpedo_main PRIVATE
    -Wall
    -Wextra
)

find_package(Threads REQUIRED)
target_link_libraries(torpedo_main PRIVATE Threads::Threads m)

add_executable(allocator_test
    tests/test_allocator.c
    src/common_Control/actuators.c
    src/common_Control/transform.c
    src/common_Control/actuator_model.c
    src/common_Control/allocator.c
    src/common_Control/vehicle_config.c
)

target_include_directories(allocator_test PRIVATE
    ${CMAKE_SOURCE_DIR}/src
)

target_compile_options(allocator_test PRIVATE
    -Wall
    -Wextra
)

target_link_libraries(allocator_test PRIVATE m)

get_filename_component(PARENT_DIR ${CMAKE_SOURCE_DIR} DIRECTORY)

add_custom_command(TARGET torpedo_main POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
        $<TARGET_FILE:torpedo_main>
        ${PARENT_DIR}/torpedo_main
)
