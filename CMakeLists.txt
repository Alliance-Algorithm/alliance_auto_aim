cmake_minimum_required(VERSION 3.22)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_BUILD_TYPE "Debug")

project(alliance_auto_aim VERSION 0.0.1 LANGUAGES CXX)

include_directories(./include ./src)

### Target lib ===
add_library(${PROJECT_NAME} SHARED)
file(GLOB_RECURSE sources_v1 CONFIGURE_DEPENDS 
src/v1/*.cpp 
src/v1/*.cc
src/core/*.cpp
src/util/*.cpp)
target_sources(
    ${PROJECT_NAME}  PUBLIC
    ${sources_v1}
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

### TBB ===
find_package(TBB REQUIRED)
include_directories(${TBB_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${TBB_LIBRARIES})
### TBB End ===

### OpenVINO ===
find_package(OpenVINO REQUIRED)
target_link_libraries(${PROJECT_NAME} openvino::runtime)
### OpenVINO End ===

### ceres ===
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${CERES_LIBRARIES})
### ceres End ===


# 安装库文件
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

install(TARGETS ${PROJECT_NAME}
EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

