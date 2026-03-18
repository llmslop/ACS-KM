cmake_minimum_required(VERSION 3.20)

project(acs_km_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(acs_km_core
    cpp/src/data_reader.cpp
    cpp/src/inout.cpp
    cpp/src/km_match.cpp
    cpp/src/parse.cpp
    cpp/src/vrptw_utils.cpp
)

target_include_directories(acs_km_core PUBLIC cpp/include)

if (MSVC)
  target_compile_options(acs_km_core PRIVATE /W4 /permissive-)
else()
  target_compile_options(acs_km_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

add_executable(acs_km_cli cpp/src/main.cpp)
target_link_libraries(acs_km_cli PRIVATE acs_km_core)

include(CTest)
if (BUILD_TESTING)
  add_test(NAME acs_km_cli_help_test COMMAND acs_km_cli --help)
  set_tests_properties(acs_km_cli_help_test PROPERTIES
    PASS_REGULAR_EXPRESSION "Usage: acs_km_cli")

  add_test(NAME acs_km_cli_usage_error_test COMMAND acs_km_cli --unexpected-option)
  set_tests_properties(acs_km_cli_usage_error_test PROPERTIES WILL_FAIL TRUE)

  add_executable(acs_km_data_reader_test cpp/tests/test_data_reader.cpp)
  target_link_libraries(acs_km_data_reader_test PRIVATE acs_km_core)
  target_compile_definitions(acs_km_data_reader_test PRIVATE ACS_KM_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
  add_test(NAME acs_km_data_reader_test COMMAND acs_km_data_reader_test)

  add_executable(acs_km_km_match_test cpp/tests/test_km_match.cpp)
  target_link_libraries(acs_km_km_match_test PRIVATE acs_km_core)
  add_test(NAME acs_km_km_match_test COMMAND acs_km_km_match_test)

  add_executable(acs_km_vrptw_utils_test cpp/tests/test_vrptw_utils.cpp)
  target_link_libraries(acs_km_vrptw_utils_test PRIVATE acs_km_core)
  add_test(NAME acs_km_vrptw_utils_test COMMAND acs_km_vrptw_utils_test)

  add_executable(acs_km_parse_inout_test cpp/tests/test_parse_inout.cpp)
  target_link_libraries(acs_km_parse_inout_test PRIVATE acs_km_core)
  add_test(NAME acs_km_parse_inout_test COMMAND acs_km_parse_inout_test)
endif()
