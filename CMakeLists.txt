cmake_minimum_required(VERSION 3.16)
project(truck_model VERSION 2.8.1 LANGUAGES CXX)

add_library(truck_model
    src/articulated_vehicle.cpp
    src/articulation_estimator.cpp
    src/lateral_mpc.cpp
    src/linear_discretization.cpp
    src/reference_path.cpp
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

option(TRUCK_MODEL_BUILD_DEMO "Build the real-time MPC visualization demo" ON)
if(TRUCK_MODEL_BUILD_DEMO)
    add_library(truck_mpc_demo_core
        examples/mpc_demo/articulation_reference.cpp
        examples/mpc_demo/demo_paths.cpp
        examples/mpc_demo/demo_session.cpp
        examples/mpc_demo/path_editor.cpp
        examples/mpc_demo/session_log.cpp
    )
    target_link_libraries(truck_mpc_demo_core PUBLIC truck_model)
    target_include_directories(truck_mpc_demo_core PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/examples/mpc_demo
    )

    if(BUILD_TESTING)
        add_executable(truck_mpc_demo_tests tests/test_mpc_demo.cpp)
        target_link_libraries(truck_mpc_demo_tests PRIVATE truck_mpc_demo_core)
        add_test(NAME truck_mpc_demo_tests COMMAND truck_mpc_demo_tests)
    endif()

    if(WIN32)
        include(FetchContent)
        FetchContent_Declare(
            dear_imgui
            URL https://codeload.github.com/ocornut/imgui/tar.gz/refs/tags/v1.92.9
            DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        )
        FetchContent_Declare(
            implot_source
            URL https://codeload.github.com/epezent/implot/tar.gz/refs/tags/v0.17
            DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        )
        FetchContent_MakeAvailable(dear_imgui implot_source)

        add_library(truck_demo_imgui STATIC
            ${dear_imgui_SOURCE_DIR}/imgui.cpp
            ${dear_imgui_SOURCE_DIR}/imgui_draw.cpp
            ${dear_imgui_SOURCE_DIR}/imgui_tables.cpp
            ${dear_imgui_SOURCE_DIR}/imgui_widgets.cpp
            ${dear_imgui_SOURCE_DIR}/backends/imgui_impl_win32.cpp
            ${dear_imgui_SOURCE_DIR}/backends/imgui_impl_dx11.cpp
        )
        target_include_directories(truck_demo_imgui PUBLIC
            ${dear_imgui_SOURCE_DIR}
            ${dear_imgui_SOURCE_DIR}/backends
        )
        target_link_libraries(truck_demo_imgui PUBLIC d3d11 dxgi)

        add_library(truck_demo_implot STATIC
            ${implot_source_SOURCE_DIR}/implot.cpp
            ${implot_source_SOURCE_DIR}/implot_items.cpp
        )
        target_include_directories(truck_demo_implot PUBLIC
            ${implot_source_SOURCE_DIR}
        )
        target_link_libraries(truck_demo_implot PUBLIC truck_demo_imgui)

        add_executable(truck_mpc_demo WIN32
            examples/mpc_demo/demo_imgui.cpp
            examples/mpc_demo/telemetry_panel.cpp
        )
        target_link_libraries(
            truck_mpc_demo PRIVATE
            truck_mpc_demo_core
            truck_demo_imgui
            truck_demo_implot
            user32
            d3d11
            dxgi
        )
        target_compile_definitions(truck_mpc_demo PRIVATE UNICODE _UNICODE)
        if(MSVC)
            target_compile_options(truck_mpc_demo PRIVATE /W4 /permissive- /utf-8)
        endif()
    else()
        message(STATUS "truck_mpc_demo is currently available on Windows only")
    endif()
endif()
