# Linear MPC - Dear ImGui Setup with Wrapper

This directory contains a C++ library structure incorporating a custom Dear ImGui wrapper (`imgui_wrapper`), a generic OSQP solver wrapper (`osqp_impl`), and an interactive MPC visualization application (`mpc`).

## Project Structure

```
linear_mpc/
  CMakeLists.txt            # Root build file
  README.md                 # Project guide
  imgui_wrapper/            # Custom wrapper component
    CMakeLists.txt          # Wrapper sub-build file
    include/
      imgui_wrapper.h       # Wrapper public class interface (rendering & inputs)
    src/
      imgui_wrapper.cpp     # Wrapper implementation (toggle buttons, mouse position)
      main.cpp              # Test script verifying mouse events & drawing
      imgui/                # Original copy-pasted Dear ImGui distribution
    bin/
      wrapper_demo          # Compiled test executable
  osqp_impl/                # Generic OSQP solver wrapper component
    CMakeLists.txt          # Solver sub-build file
    include/
      osqp_solver.h         # Solver wrapper public header
    src/
      osqp_solver.cpp       # Solver wrapper implementation
    bin/
      solver_demo           # Test executable solving a 2D QP
  mpc/                      # MPC visualization & simulation
    CMakeLists.txt          # MPC sub-build file
    bin/
      test_mpc.cpp          # Interactive tracking demo (draggable target, following state)
      mpc_demo              # Compiled interactive demo executable
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

## Running the MPC Interactive Demo

To test the interactive circle tracking simulation:
```bash
./mpc/bin/mpc_demo
```
* **Dragging**: Left-click and hold near the light red target circle to drag it around.
* **Toggle**: Toggle the "Run MPC Loop" checkbox at the top-left:
  * When **disabled**: Moving the target has no effect on the current state.
  * When **enabled**: The navy blue circle follows the target circle.
