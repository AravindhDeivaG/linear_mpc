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
