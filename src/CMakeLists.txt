find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(vio_executable main.cpp camera.cpp)

set_target_properties(vio_executable PROPERTIES
RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)
target_link_libraries(vio_executable PUBLIC ${OpenCV_LIBS})
# Link OpenCV libraries and include directories
target_include_directories(vio_executable PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(vio_executable PUBLIC ${OpenCV_LIBS} opencv_xfeatures2d) # <-- ADD THIS MODULE
target_include_directories(vio_executable PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}) # <-- ADDED THIS
