cmake_minimum_required(VERSION 3.16)
project(truck_model VERSION 2.9.0 LANGUAGES CXX)

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

option(TRUCK_MODEL_BUILD_DEMO "Build the MPC demo session library and its tests" ON)
# The Studio executable needs Win32, DirectX 11 and a network fetch of
# ImGui/ImPlot. Turn it off to build and test the headless demo core only.
option(TRUCK_MODEL_BUILD_STUDIO "Build the ImGui MPC Studio executable" ON)
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

    if(WIN32 AND TRUCK_MODEL_BUILD_STUDIO)
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
        # MSVC picks d3dcompiler and dwmapi up from #pragma comment(lib) inside
        # the ImGui backends; other toolchains ignore those pragmas and need the
        # libraries named explicitly.
        target_link_libraries(truck_demo_imgui PUBLIC
            d3d11
            dxgi
            d3dcompiler
            dwmapi
        )

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
        elseif(MINGW)
            # The entry point is wWinMain, which MinGW only emits with -municode.
            target_compile_options(truck_mpc_demo PRIVATE -municode)
            target_link_options(truck_mpc_demo PRIVATE -municode)
        endif()
    elseif(NOT TRUCK_MODEL_BUILD_STUDIO)
        message(STATUS "truck_mpc_demo disabled by TRUCK_MODEL_BUILD_STUDIO=OFF")
    else()
        message(STATUS "truck_mpc_demo is currently available on Windows only")
    endif()
endif()
