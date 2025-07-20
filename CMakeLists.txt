cmake_minimum_required(VERSION 3.22)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE "Release")

project(alliance_auto_aim VERSION 0.0.1 LANGUAGES CXX)

include_directories(./include)

### Target lib ===
add_library(${PROJECT_NAME} 
src/core/alliance.cpp
)
### Target lib End ===

### OpenCV ===
# set(OpenCV_DIR /path/to/opencv/build) # 如果需要这个可以方便你设置
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}  ${OpenCV_LIBS})
### OpenCV End ===

### Eigen ===
find_package(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
### Eigen End ===

target_include_directories(${PROJECT_NAME}  PUBLIC
    ./include
)

install(DIRECTORY include/ DESTINATION include)

