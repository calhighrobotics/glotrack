cmake_minimum_required(VERSION 4.4.0)

project(glotrack)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)
include_directories(imgui implot)

#make an imgui folder and put all of these files in the imgui folder
set(IMGUI_SRC
        imgui/backends/imgui_impl_opengl3.h
        imgui/backends/imgui_impl_opengl3.cpp
        imgui/backends/imgui_impl_opengl3_loader.h
        imgui/backends/imgui_impl_glfw.h
        imgui/backends/imgui_impl_glfw.cpp
        imgui/imgui.h
        imgui/imgui.cpp
        imgui/imgui_draw.cpp
        imgui/imgui_tables.cpp
        imgui/imgui_widgets.cpp
        imgui/imgui_demo.cpp
        )

set(IMPLOT_SRC
        implot/implot.h
        implot/implot_internal.h
        implot/implot.cpp
        implot/implot_items.cpp
        )

add_executable(glotrack ${IMGUI_SRC} ${IMPLOT_SRC})

target_sources(glotrack
    PRIVATE
        src/main.cpp
        src/KalmanFilter.cpp
        src/PoseLogger.cpp
        src/LocalizationPipeline.cpp
        src/util.cpp
)
target_link_libraries(glotrack PRIVATE OpenGL::OpenGL glfw ${OpenCV_LIBS})