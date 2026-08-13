cmake_minimum_required(VERSION 3.16)
project(truck_model VERSION 1.0.0 LANGUAGES CXX)

add_library(truck_model
    src/articulated_vehicle.cpp
)
target_include_directories(truck_model PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(truck_model PUBLIC cxx_std_17)

if(MSVC)
    target_compile_options(truck_model PRIVATE /W4 /permissive-)
else()
    target_compile_options(truck_model PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(truck_model_tests tests/test_articulated_vehicle.cpp)
    target_link_libraries(truck_model_tests PRIVATE truck_model)
    add_test(NAME truck_model_tests COMMAND truck_model_tests)
endif()
