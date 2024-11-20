cmake_minimum_required(VERSION 3.29)
project(simulate_auton_path)

set(CMAKE_CXX_STANDARD 20)

file(GLOB_RECURSE HEADERS "include/*.h" "include/*.hpp")
file(GLOB_RECURSE SOURCES "src/*.cpp")


include_directories("include/")
# Add executable with all source files
add_executable(${PROJECT_NAME} ${SOURCES}
        src/
        include/aux.h
        src/aux_task.cpp
        src/generate_jerryio.cpp
        include/generate_jerryio.h
        src/simulating_code.cpp
        include/simulating_code.h)