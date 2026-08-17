# Sparse MPC Formulation

The Sparse (or Full-Space) MPC formulation retains state variables $x_k$ explicitly in the decision vector alongside control inputs $u_k$. Rather than condensing states out, dynamic transition equations are enforced as linear equality constraints inside the QP solver.

---

## 1. Interleaved Decision Vector ($z$)

For a prediction horizon $N$, state dimension $n_s$, and input dimension $n_i$, the decision vector $z$ interleaves control inputs and state predictions:

$$
z = {\begin{bmatrix}
u_0 \\\\
x_1 \\\\
u_1 \\\\
x_2 \\\\
\vdots \\\\
u_{N-1} \\\\
x_N
\end{bmatrix}}_{N(n_s + n_i) \times 1}
$$

---

## 2. Selection Matrices ($S_x$ and $S_u$)

Selection matrices extract stacked states $X_{\text{stacked}}$ and stacked inputs $U_{\text{stacked}}$ directly from $z$:

$$
X_{\text{stacked}} = S_x z = {\begin{bmatrix}
x_1 \\\\
x_2 \\\\
\vdots \\\\
x_N
\end{bmatrix}}_{N n_s \times 1}, \quad
U_{\text{stacked}} = S_u z = {\begin{bmatrix}
u_0 \\\\
u_1 \\\\
\vdots \\\\
u_{N-1}
\end{bmatrix}}_{N n_i \times 1}
$$

Where:

* $S_x \in \mathbb{R}^{(N n_s) \times N(n_s + n_i)}$ has block identity matrices picking $x_{k+1}$.
* $S_u \in \mathbb{R}^{(N n_i) \times N(n_s + n_i)}$ has block identity matrices picking $u_k$.

---

## 3. Quadratic Cost Function

The objective function penalizes state tracking errors and control inputs:

$$
J = (S_x z - X_{ref})^T \mathbf{Q} (S_x z - X_{ref}) + (S_u z)^T \mathbf{R} (S_u z)
$$

Expanding into standard OSQP form $\frac{1}{2} z^T H z + g^T z$:

$$
H = 2(S_x^T \mathbf{Q} S_x + S_u^T \mathbf{R} S_u)
$$

$$
g = -2 S_x^T \mathbf{Q} X_{ref}
$$

> **Key Benefit**: Because $S_x$ and $S_u$ select distinct elements of $z$, the Hessian $H$ is **block-diagonal**, eliminating dense matrix products during setup.

---

## 4. Sparse Constraint Assembly ($l \le M z \le u$)

The total constraint matrix $M$ has dimensions $N(2 n_s + n_i) \times N(n_s + n_i)$ and contains two main blocks:

### A. Variable Bounds (Top Block)
Enforces lower and upper bounds on $u_k$ and $x_k$ using an Identity matrix $I_{N(n_s + n_i) \times N(n_s + n_i)}$:

$$
l_{\text{box}} = {\begin{bmatrix}
u_{\min} \\\\
x_{\min} \\\\
u_{\min} \\\\
x_{\min} \\\\
\vdots
\end{bmatrix}}_{N(n_s + n_i) \times 1}, \quad
u_{\text{box}} = {\begin{bmatrix}
u_{\max} \\\\
x_{\max} \\\\
u_{\max} \\\\
x_{\max} \\\\
\vdots
\end{bmatrix}}_{N(n_s + n_i) \times 1}
$$

### B. Dynamic Transition Equality Constraints (Bottom Block)
Enforces state propagation equations $x_{k+1} = A x_k + B u_k$:

1. **Step $k=0$**: $B u_0 - x_1 = -A x_0$
2. **Step $k \ge 1$**: $A x_k + B u_k - x_{k+1} = 0$

---

## 5. 3-Horizon Matrix Assembly Example ($N = 3$)

For $N = 3$, the constraint matrix $M$ is block-tridiagonal:

$$
M = {\begin{bmatrix}
I & 0 & 0 & 0 & 0 & 0 \\\\
0 & I & 0 & 0 & 0 & 0 \\\\
0 & 0 & I & 0 & 0 & 0 \\\\
0 & 0 & 0 & I & 0 & 0 \\\\
0 & 0 & 0 & 0 & I & 0 \\\\
0 & 0 & 0 & 0 & 0 & I \\\\
B & -I & 0 & 0 & 0 & 0 \\\\
0 & A & B & -I & 0 & 0 \\\\
0 & 0 & 0 & A & B & -I
\end{bmatrix}}_{(3(2 n_s + n_i)) \times 3(n_s + n_i)}
$$

Lower bound $l$ and upper bound $u$:

$$
l = \begin{bmatrix}
u_{\min} \\\\
x_{\min} \\\\
u_{\min} \\\\
x_{\min} \\\\
u_{\min} \\\\
x_{\min} \\\\
-A x_0 \\\\
0 \\\\
0
\end{bmatrix}, \quad
u = \begin{bmatrix}
u_{\max} \\\\
x_{\max} \\\\
u_{\max} \\\\
x_{\max} \\\\
u_{\max} \\\\
x_{\max} \\\\
-A x_0 \\\\
0 \\\\
0
\end{bmatrix}
$$

---

## 6. Implementation & Solver Optimizations

To ensure mathematical consistency with the Dense formulation and achieve real-time performance, the following optimizations are implemented in `SparseFormulation`:

1. **Diagonal Variable Pre-conditioning & Scaling**:
   State and input variables are normalized into the unit hypercube $[-1, 1]$ using diagonal scaling matrices $T_x$ and $T_u$. System dynamics matrices are transformed as $\hat{A} = T_x A T_x^{-1}$ and $\hat{B} = T_x B T_u^{-1}$, while cost matrices become $\hat{Q} = T_x^{-1} Q T_x^{-1}$ and $\hat{R} = T_u^{-1} R T_u^{-1}$. This reduces the KKT matrix condition number from $2.5 \times 10^7$ down to $\sim 1.0$, preventing OSQP solver stall under high-velocity dynamics.

2. **OSQP Matrix Factorization Caching**:
   Hessian ($H$) and constraint ($M$) matrices are passed to OSQP once during `setup()`. During control execution, only vectors $g$, $l$, and $u$ are updated using `osqp_update_data_vec()`, bypassing `osqp_update_data_mat()`. This enables OSQP to reuse its pre-factorized $LDL^T$ linear solver factorization across consecutive solves, reducing execution time from milliseconds to sub-microseconds.

3. **ADMM Tolerance Calibration**:
   The OSQP stopping criteria are calibrated to `eps_abs = 1e-5` and `eps_rel = 1e-5` for the scaled formulation. On the normalized scale, this guarantees sub-millimeter force precision ($< 10^{-5}\text{ N}$) without exceeding maximum iteration limits when inputs hit active bounds.

4. **Zero-Curvature Regularization**:
   Control cost matrices are strictly regularized with $R \ge 10^{-3} \cdot I$ to guarantee strict positive-definiteness ($H \succ 0$). This removes zero-curvature null-spaces along control dimensions, ensuring a unique global minimum.

5. **In-Place Sparse Matrix Iteration**:
   Sparse constraint updates use `Eigen::SparseMatrix::InnerIterator` pattern to write non-zero entries directly into OSQP's Column-Compressed Sparse (CSC) memory buffers without triggering dynamic allocations or array reallocation.
