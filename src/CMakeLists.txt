# 
# Define the executable and its source files
add_executable(vio_executable
    main.cpp
    camera.cpp
    logger.cpp
)

# Add include directory for local headers (.hpp files)
target_include_directories(vio_executable PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Link all necessary libraries in a single, clean command
target_link_libraries(vio_executable PRIVATE 
    ${OpenCV_LIBS}  
    opencv_xfeatures2d    # From find_package(OpenCV) in the parent file
)