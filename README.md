# Linear MPC Visualizer & Strategy Framework

This project is an interactive 2D tracking simulator that visualizes a double-integrator point mass tracking a draggable target in real-time. The controller is powered by a modular, strategy-pattern **Model Predictive Control (MPC)** engine solved via the **OSQP** quadratic programming solver.

The predicted trajectory planned by the MPC solver is rendered on-screen as a sequence of bright cyan hollow circles connecting the current state (navy blue circle) to the target reference (semi-transparent red circle).

![MPC Tracking Demo](imgs/mpc_demo.gif)

---

## Key Features & Architecture

* **Swappable Formulation Strategies**: Utilizes a Strategy pattern allowing dynamic switching between **Dense (Condensed)** and **Sparse (Full-Space)** MPC backends (`MpcController`).
* **Zero Runtime Dynamic Allocation**: Problem dimensions ($N, n_s, n_i$) are declared at instantiation to pre-allocate all matrices upfront.
* **Problem-Agnostic Engine**: System dynamics matrices ($A, B$), state bounds ($x_{\min}, x_{\max}$), input bounds ($u_{\min}, u_{\max}$), and cost weights ($Q, R$) are configured externally by the user application.

---

## Installation

### 1. System Dependencies
Ensure the following development libraries are installed on your system:
* **GLFW3** & **OpenGL** (for real-time rendering)
* **Eigen3** (for matrix algebra)

On Ubuntu/Debian, install them via:
```bash
sudo apt update
sudo apt install build-essential cmake libglfw3-dev libgl1-mesa-dev libx11-dev
```

### 2. Submodule
* **Dear ImGui**: Bundled locally within `imgui_wrapper/src/imgui`. No external installation is needed.

### 3. OSQP Solver (v1.0.0)
The MPC controller requires **OSQP version 1.0.0** installed on your system.

```bash
git clone --recursive https://github.com/osqp/osqp.git
cd osqp
git checkout v1.0.0
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

### 4. Building the Project
To compile the library and executable targets:
```bash
cmake -B build -S .
cmake --build build
```

Executables will be generated at:
* `./build/mpc/mpc_viz` (Interactive 2D ImGui visualizer)
* `./build/mpc/mpc_demo` (Math validation & strategy comparison test)

---

## Formulations & Mathematical Documentation

The framework supports two distinct MPC formulation strategies:

### 1. Dense (Condensed) Formulation
Eliminates intermediate state predictions by expressing future states as $X = S_x x_0 + S_u U$. Solves a smaller QP problem in $U \in \mathbb{R}^{N n_i}$ with dense Hessian $H = 2(S_u^T \mathbf{Q} S_u + \mathbf{R})$.

📖 **[Read Full Dense Formulation Guide](docs/dense_formulation.md)**

### 2. Sparse (Full-Space) Formulation
Interleaves inputs and states inside the decision vector $z = [u_0^T, x_1^T, u_1^T, x_2^T, \dots]^T \in \mathbb{R}^{N(n_s + n_i)}$. Retains diagonal Hessian $H = 2(S_x^T \mathbf{Q} S_x + S_u^T \mathbf{R} S_u)$ and enforces system dynamics $x_{k+1} = A x_k + B u_k$ as block-tridiagonal linear equality constraints.

📖 **[Read Full Sparse Formulation Guide](docs/sparse_formulation.md)**

---

## Key Performance Diagnostics & Resolved Issues

### 1. Sparse Matrix Zero-Entry Inflation
* **Issue**: OSQP is designed as a sparse solver by default, relying on Compressed Sparse Column (CSC) matrix structures to process only non-zero entries. Initially, explicit `0.0` values were being written to the sparse matrices (`m_P_sparse` and `m_A_sparse`) alongside structural non-zeros. This turned true sparse matrices (~200 non-zero entries) into dense arrays (6,000 non-zero entries), forcing OSQP to perform 30 times more matrix operations per iteration and slowing down solver execution dramatically.
* **Fix**: Updated matrix initialization loops to explicitly filter out zero values (`std::abs(val) > 1e-12`), writing only true non-zero entries into the sparse structures. Iteration execution speed dropped drastically into the target microsecond range.

### 2. Residual Convergence Imbalance (Primal vs Dual Error)
* **Issue**: Under high-velocity motion near active bounds, the dual residual converged very quickly while the primal residual converged slowly, causing ADMM iterations to stall or hit maximum iteration limits.
* **Fix**: Optimized the ADMM penalty step-size parameter $\rho$. Increasing $\rho$ penalizes primal constraint violations more heavily per iteration, accelerating primal residual reduction so that optimal convergence is reached much faster.

### 3. Dense Formulation Sensitivity to Fixed Step-Size ($\rho$)
* **Issue**: Dense formulation matrices have full, non-diagonal Hessian structures compared to sparse formulations. When running Dense MPC with a fixed step-size (`adaptive_rho: false`), the primal residual hits tolerance (< 1e-3) instantly while the dual residual stagnates at a large value (~60-70), causing OSQP to stall for thousands of iterations. Sparse formulation converges robustly under both fixed and adaptive $\rho$.
* **Fix**: Enabled `adaptive_rho: true` in `osqp_params.yaml`. Adaptive $\rho$ dynamically rebalances the step-size parameter based on the ratio of primal to dual residuals ($r_{\text{prim}} / r_{\text{dual}}$), collapsing the dual residual and enabling Dense MPC to converge in ~20 iterations.

---

## Usage Example

```cpp
#include "mpc_controller.h"

// 1. Instantiate controller with horizon N=20, 4 states, 2 inputs
MpcController mpc(20, 4, 2, FormulationType::SPARSE);

// 2. Set system dynamics (A, B) and constraints
mpc.setSystemMatrices(A, B);
mpc.setStateLimits(x_min, x_max);
mpc.setInputLimits(u_min, u_max);
mpc.setCostMatrices(Q, R);

// 3. Populate pre-allocated matrices & setup solver workspace
mpc.setup();

// 4. Control loop execution
mpc.setCurrentState(x_current);
mpc.setReferenceState(x_target);
mpc.doControl();

// 5. Retrieve optimal control action u0 and predicted trajectory X
Eigen::VectorXd u0, X_pred;
mpc.getOptimalControl(u0);
mpc.getPredictedStates(X_pred);
```
