cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 包含主项目头文件
include_directories(../include)

add_executable(test_main test_main.cpp)

# 确保测试使用 C++20
target_compile_features(test_main PRIVATE cxx_std_20)

# GoogleTest 配置
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/heads/main.zip
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

target_link_libraries(test_main 
    GTest::gtest_main 
    GTest::gmock_main
)

include(GoogleTest)
gtest_discover_tests(test_main)