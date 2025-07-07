# Minimum required CMake version
cmake_minimum_required(VERSION 3.10)

# Project name
project(VIO_Project LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add a source directory (where your C++ files will be)
# We'll create a 'src' directory for our source code
add_subdirectory(src)

# --- NEW: Find OpenCV ---
find_package(OpenCV REQUIRED) # REQUIRED means CMake will fail if OpenCV is not found

# If you get errors about OpenCV not being found, you might need to
# set OpenCV_DIR to the directory containing OpenCVConfig.cmake or opencv-config.cmake
# Example (uncomment and adjust if needed):
# set(OpenCV_DIR "/path/to/your/opencv/build")
# find_package(OpenCV REQUIRED)
# -------------------------

# Set output directories for executables and libraries
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)