cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 包含主项目头文件
include_directories(../include)

file(GLOB_RECURSE TONGJI_TEST_SOURCES "./tongji_tests/*.cpp")

add_executable(all_tests 
    ${TONGJI_TEST_SOURCES})

# 确保测试使用 C++20
target_compile_features(all_tests PRIVATE cxx_std_20)

# GoogleTest 配置
include(FetchContent)
FetchContent_Declare(
  googletest
  SOURCE_DIR ${CMAKE_SOURCE_DIR}/thirdparty/googletest
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

target_link_libraries(all_tests 
    GTest::gtest_main 
    GTest::gmock_main
    alliance_auto_aim
)

include(GoogleTest)
gtest_discover_tests(all_tests)
