cmake_minimum_required(VERSION 3.20)

project(acs_km_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(acs_km_core
    cpp/src/data_reader.cpp
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
  add_executable(acs_km_data_reader_test cpp/tests/test_data_reader.cpp)
  target_link_libraries(acs_km_data_reader_test PRIVATE acs_km_core)
  target_compile_definitions(acs_km_data_reader_test PRIVATE ACS_KM_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
  add_test(NAME acs_km_data_reader_test COMMAND acs_km_data_reader_test)
endif()
