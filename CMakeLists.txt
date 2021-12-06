cmake_minimum_required(VERSION 3.10)

project("spaceryde")

set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 20)

add_executable(spaceryde main.cpp)

target_include_directories(spaceryde PRIVATE .)