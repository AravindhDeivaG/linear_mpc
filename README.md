# Linear MPC - Dear ImGui Setup with Wrapper

This directory contains a main C++ project structure and a custom wrapper library (`imgui_wrapper`) to simplify windowing, input events, and drawing shapes.

## Project Structure

```
linear_mpc/
  CMakeLists.txt            # Root build file (only includes the subdirectory)
  README.md                 # Project guide
  imgui_wrapper/            # Custom wrapper component
    CMakeLists.txt          # Wrapper sub-build file (contains all ImGui compilation targets)
    include/
      imgui_wrapper.h       # Wrapper public class interface
    src/
      imgui_wrapper.cpp     # Wrapper implementation (lifecycle, events, circles)
      main.cpp              # Test script verifying mouse events & drawing
      imgui/                # Original copy-pasted Dear ImGui distribution
        backends/           # Renderer and platform bindings (GLFW, OpenGL, etc.)
    bin/
      wrapper_demo          # Compiled test executable (automatically generated)
```

## Prerequisites

To build and run this project, make sure you have the required development headers installed on your system.

On **Ubuntu / Debian**:
```bash
sudo apt-get update
sudo apt-get install build-essential cmake libglfw3-dev libgl1-mesa-dev libx11-dev
```

## How to Build

1. Create a build directory:
   ```bash
   mkdir build
   cd build
   ```

2. Configure and compile:
   ```bash
   cmake ..
   cmake --build .
   ```

## How to Run

After compilation, the demonstration application is output directly into `imgui_wrapper/bin/`:
```bash
./imgui_wrapper/bin/wrapper_demo
```

### Demonstration Script Behavior
- When you **click and hold** the left mouse button, a blue solid circle (radius 15) and a red hollow circle (radius 30) are drawn on the screen centered at the mouse coordinates.
- When you **release** the left mouse button, the circles disappear.
