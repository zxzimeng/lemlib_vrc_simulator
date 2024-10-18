cmake_minimum_required(VERSION 3.29)
project(arc_length_cpp)

set(CMAKE_CXX_STANDARD 20)

add_executable(arc_length_cpp main.cpp)

add_executable(compile_time_arclength compile_time_arclength.cpp)