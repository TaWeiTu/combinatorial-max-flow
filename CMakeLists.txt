cmake_minimum_required(VERSION 3.30)
project(CombinatorialMaximumFlow LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED True)
add_compile_options(-Wall -Wextra -Wpedantic)

if(NOT (APPLE AND CMAKE_SYSTEM_PROCESSOR STREQUAL "arm64"))
  if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-fsanitize=undefined)
    add_link_options(-fsanitize=undefined)
  endif()
endif()

file(GLOB_RECURSE TEST_SRC_FILES CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/tests/*.cc")
file(GLOB_RECURSE SRC_FILES CONFIGURE_DEPENDS "${PROJECT_SOURCE_DIR}/src/*.cc")

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
catch_discover_tests(tests TEST_SPEC ~[stress])
catch_discover_tests(tests TEST_SPEC [stress] PROPERTIES LABELS stress)
list(APPEND CMAKE_CTEST_ARGUMENTS "--output-on-failure")
