# 
cmake_minimum_required(VERSION 3.10)
project(VIO_Project LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find packages at the top level so they are available to all subdirectories
find_package(OpenCV REQUIRED)

# Add the external library FIRST to make its target available
# Add the source directory LAST, now it can find the yaml-cpp target
add_subdirectory(src) # <--- MOVED DOWN

# Set output directories for all targets in the project
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)