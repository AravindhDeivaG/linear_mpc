# Dense MPC Formulation

The Dense (or Condensed) MPC formulation eliminates intermediate state variables over the prediction horizon by expressing future states as explicit functions of the initial state $x_0$ and control inputs $U$.

---

## 1. System Dynamics

For a discrete-time linear system:

$$
x_{k+1} = A x_k + B u_k
$$

where $x_k \in \mathbb{R}^{n_s}$ is the state vector and $u_k \in \mathbb{R}^{n_i}$ is the control input vector.

For a 2D double-integrator point mass with time step $dt$:

$$
A = \begin{bmatrix}
1 & 0 & dt & 0 \\\\
0 & 1 & 0 & dt \\\\
0 & 0 & 1 & 0 \\\\
0 & 0 & 0 & 1
\end{bmatrix}, \quad
B = \begin{bmatrix}
0.5 dt^2 & 0 \\\\
0 & 0.5 dt^2 \\\\
dt & 0 \\\\
0 & dt
\end{bmatrix}
$$

---

## 2. Condensed Model Formulation

Over a prediction horizon $N$, the predicted state sequence $X$ is expressed as:

$$
X = S_x x_0 + S_u U
$$

where the stacked state and control vectors are:

$$
X = {\begin{bmatrix}
x_1 \\\\
x_2 \\\\
\vdots \\\\
x_N
\end{bmatrix}}_{N n_s \times 1}, \quad
U = {\begin{bmatrix}
u_0 \\\\
u_1 \\\\
\vdots \\\\
u_{N-1}
\end{bmatrix}}_{N n_i \times 1}
$$

---

## 3. 3-Horizon Matrix Assembly ($N = 3$)

For a horizon of $N = 3$, the dynamic propagation matrices $S_x$ and $S_u$ are assembled as:

$$
S_x = {\begin{bmatrix}
A \\\\
A^2 \\\\
A^3
\end{bmatrix}}_{3 n_s \times n_s}
$$

$$
S_u = {\begin{bmatrix}
B & 0 & 0 \\\\
AB & B & 0 \\\\
A^2 B & AB & B
\end{bmatrix}}_{3 n_s \times 3 n_i}
$$

---

## 4. Quadratic Program (QP) Optimization

The objective function penalizes tracking error and control effort:

$$
J = (X - X_{ref})^T \mathbf{Q} (X - X_{ref}) + U^T \mathbf{R} U
$$

where $\mathbf{Q} = \text{diag}(Q, \dots, Q)$ and $\mathbf{R} = \text{diag}(R, \dots, R)$ are block-diagonal weight matrices. 

Substituting $X = S_x x_0 + S_u U$ converts the problem into standard OSQP form $\frac{1}{2} U^T H U + g^T U$:

$$
H = 2(S_u^T \mathbf{Q} S_u + \mathbf{R})
$$

$$
g = 2 S_u^T \mathbf{Q} (S_x x_0 - X_{ref})
$$

### Constraints ($l \le M U \le u$)

Control input and state bounds are combined into a single matrix inequality:

$$
M = \begin{bmatrix}
I_{N n_i \times N n_i} \\\\
S_u
\end{bmatrix}, \quad
l = \begin{bmatrix}
U_{\min} \\\\
-S_x x_0 + X_{\min}
\end{bmatrix}, \quad
u = \begin{bmatrix}
U_{\max} \\\\
-S_x x_0 + X_{\max}
\end{bmatrix}
$$

For $N = 3$, the constraint matrix $M$ has dimensions $(3 n_i + 3 n_s) \times 3 n_i$:

$$
M = {\begin{bmatrix}
I_{3 n_i \times 3 n_i} \\\\
S_u
\end{bmatrix}}_{(3 n_i + 3 n_s) \times 3 n_i}
$$
