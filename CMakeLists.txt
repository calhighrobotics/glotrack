cmake_minimum_required(VERSION 4.4.0)

project(glotrack
    VERSION 0.1.0
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(OpenCV REQUIRED COMPONENTS
    core
    calib
    imgproc
    imgcodecs
    videoio
    video
    flann
    objdetect
)

find_package(OpenGL REQUIRED)
find_package(glfw3 CONFIG REQUIRED)

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

add_executable(glotrack
    ${IMGUI_SRC}
    ${IMPLOT_SRC}
)

target_include_directories(glotrack PRIVATE
    imgui
    implot
)

target_sources(glotrack
    PRIVATE
        src/main.cpp
        src/KalmanFilter.cpp
        src/PoseLogger.cpp
        src/LocalizationPipeline.cpp
        src/util.cpp
)

target_link_libraries(glotrack PRIVATE
    OpenGL::OpenGL
    glfw
    ${OpenCV_LIBS}
)

#
# Installation
#

include(GNUInstallDirs)

install(
    TARGETS glotrack
    RUNTIME DESTINATION .
    LIBRARY DESTINATION .
    BUNDLE DESTINATION .
)

#
# Linux: allow bundled shared libraries beside executable
#

if(UNIX AND NOT APPLE)
    set_target_properties(glotrack PROPERTIES
        INSTALL_RPATH "$ORIGIN"
    )
endif()


#
# Packaging
#

include(CPack)

set(CPACK_PACKAGE_NAME "glotrack")
set(CPACK_PACKAGE_VENDOR "glotrack")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})

set(CPACK_GENERATOR
    TGZ
    ZIP
)

set(CPACK_PACKAGE_FILE_NAME
    "${PROJECT_NAME}-${PROJECT_VERSION}-${CMAKE_SYSTEM_NAME}"
)