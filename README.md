# Linear MPC - Dear ImGui Setup with Wrapper

This directory contains a C++ library structure incorporating a custom Dear ImGui wrapper (`imgui_wrapper`) and a generic OSQP solver wrapper (`osqp_impl`).

## Project Structure

```
linear_mpc/
  CMakeLists.txt            # Root build file
  README.md                 # Project guide
  imgui_wrapper/            # Custom wrapper component
    CMakeLists.txt          # Wrapper sub-build file
    include/
      imgui_wrapper.h       # Wrapper public class interface
    src/
      imgui_wrapper.cpp     # Wrapper implementation (lifecycle, events, circles)
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

## OSQP Solver Wrapper Usage

The `osqp_impl` library builds a static library `libosqp_impl.a` linking against Eigen and the OSQP solver. In your application, you can use the `OsqpSolver` class:

```cpp
#include <osqp_solver.h>
#include <Eigen/Dense>

// Initialize solver with n variables and m constraints
OsqpSolver solver(n, m);

// Set problem matrices and vectors
solver.setHessian(P);
solver.setGradient(q);
solver.setConstraintMatrix(A);
solver.setLowerBound(l);
solver.setUpperBound(u);

// Solve the QP
if (solver.solve()) {
    Eigen::VectorXd solution = solver.getSolution();
    // Do something with solution...
}
```
