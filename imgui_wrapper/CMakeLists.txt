# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
# Find OpenGL
find_package(OpenGL REQUIRED)

# Find GLFW3
find_package(glfw3 REQUIRED)

# ---------------------------------------------------------------------------
# Dear ImGui Core Library Target
# ---------------------------------------------------------------------------
add_library(imgui STATIC
    src/imgui/imgui.cpp
    src/imgui/imgui_demo.cpp
    src/imgui/imgui_draw.cpp
    src/imgui/imgui_tables.cpp
    src/imgui/imgui_widgets.cpp
)

# Export the header search path so linking targets automatically get the includes
target_include_directories(imgui PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/imgui
)

# ---------------------------------------------------------------------------
# Dear ImGui GLFW + OpenGL3 Backend Library Target
# ---------------------------------------------------------------------------
add_library(imgui_backend STATIC
    src/imgui/backends/imgui_impl_glfw.cpp
    src/imgui/backends/imgui_impl_opengl3.cpp
)

# Export backend headers search path
target_include_directories(imgui_backend PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/imgui/backends
)

# Link the backend to ImGui core, GLFW, and OpenGL
target_link_libraries(imgui_backend PUBLIC
    imgui
    glfw
    OpenGL::GL
)

# ---------------------------------------------------------------------------
# ImGui Wrapper Library Target
# ---------------------------------------------------------------------------
add_library(imgui_wrapper STATIC
    src/imgui_wrapper.cpp
)

# Expose include/ directory headers publicly
target_include_directories(imgui_wrapper PUBLIC
    include
)

# Link wrapper to the imgui_backend
target_link_libraries(imgui_wrapper PUBLIC
    imgui_backend
)

# ---------------------------------------------------------------------------
# Demo Application
# ---------------------------------------------------------------------------

# Create an executable to test the wrapper functionality
add_executable(wrapper_demo bin/test_imgui.cpp)
target_link_libraries(wrapper_demo PRIVATE imgui_wrapper)
