cmake_minimum_required(VERSION 3.16)
project(LinearMPC_DearImGui C CXX)

# Set C++ standard to C++11 (minimum required by Dear ImGui)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Subdirectories
# ---------------------------------------------------------------------------
# Include the wrapper subdirectory which encapsulates all ImGui build rules
add_subdirectory(imgui_wrapper)

# Include the generic OSQP solver wrapper subdirectory
add_subdirectory(osqp_impl)
