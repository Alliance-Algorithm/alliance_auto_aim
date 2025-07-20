set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 20)

project(alliance_auto_aim VERSION 0.0.1 LANGUAGES CXX)

# 1. 创建库目标
add_library(${PROJECT_NAME} 
    src/my_lib.cpp
)

