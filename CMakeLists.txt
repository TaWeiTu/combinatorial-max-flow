cmake_minimum_required(VERSION 3.30)
project(CombinatorialMaximumFlow LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "-Wall -Wextra -g -fsanitize=undefined")

file(GLOB TEST_SRC_FILES "tests/*.cc")
file(GLOB SRC_FILES "src/*.cc")

add_library(combflow "${SRC_FILES}")

Include(FetchContent)

FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.7.1
)

FetchContent_MakeAvailable(Catch2)

set(CMAKE_SKIP_TEST_ALL_DEPENDENCY OFF)

add_executable(tests "${TEST_SRC_FILES}")
target_link_libraries(tests PRIVATE combflow Catch2::Catch2WithMain)

list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(CTest)
include(Catch)
catch_discover_tests(tests)

set(CTEST_OUTPUT_ON_FAILURE ON)
