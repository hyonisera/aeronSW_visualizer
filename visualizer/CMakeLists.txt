#cmake version
cmake_minimum_required(VERSION 3.26)

# project name and version
project(aeronSW_visualizer VERSION 1.0)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# OpenGL, GLFW Library
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)

# opencv library
find_package(OpenCV 4.2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


# Source file and header
set(SRC_FILES
    src/main.cpp
    src/camera.cpp
    src/space.cpp
    src/binary_utils.cpp
)

set(HEADER_FILES
    src/camera.h
    src/space.h
    src/configs.h
    src/binary_utils.h
    # src/stb_image.h
)

# Execution files
add_executable(${PROJECT_NAME} ${SRC_FILES} ${HEADER_FILES})

# Link
target_link_libraries(${PROJECT_NAME} 
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    GLEW::GLEW
    glfw
    ${OpenCV_LIBS}
)

# Add the include directory
target_include_directories(${PROJECT_NAME} PRIVATE 
    src
)