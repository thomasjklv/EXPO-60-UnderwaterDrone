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

# pthread support (voor multithreading)
find_package(Threads REQUIRED)
target_link_libraries(torpedo_main PRIVATE Threads::Threads)