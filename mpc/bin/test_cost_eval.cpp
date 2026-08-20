#include "mpc_controller.h"
#include "dense_formulation.h"
#include "sparse_formulation.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <vector>
#include <Eigen/Dense>

int main() {
    const int horizon = 15;
    const int nx = 4;
    const int nu = 2;
    const double dt = 0.02;

    // 1. Define 2D double-integrator dynamics
    Eigen::MatrixXd A(nx, nx);
    A.setIdentity();
    A(0,2) = dt;
    A(1,3) = dt;

    Eigen::MatrixXd B(nx, nu);
    B.setZero();
    B(0,0) = 0.5*dt*dt;
    B(1,1) = 0.5*dt*dt;
    B(2,0) = dt;
    B(3,1) = dt;

    Eigen::VectorXd x_min(nx), x_max(nx);
    x_min << -10000, -10000, -200, -200;
    x_max <<  10000,  10000,  200,  200;

    Eigen::VectorXd u_min(nu), u_max(nu);
    u_min << -500, -500;
    u_max <<  500,  500;

    Eigen::MatrixXd Q(nx, nx);
    Q.setZero();
    Q.block(0,0,2,2) = Eigen::MatrixXd::Identity(2,2)*10.0;
    Q.block(2,2,2,2) = Eigen::MatrixXd::Identity(2,2)*0.1;

    Eigen::MatrixXd R(nu, nu);
    R = Eigen::MatrixXd::Identity(nu, nu) * 1e-3;

    // Initial state x0 and reference x_ref
    Eigen::VectorXd x0(nx);
    x0 << 219.599984, 280.400015, 139.999937, -139.999937;

    Eigen::VectorXd x_ref(nx);
    x_ref << 623.0, 80.0, 0.0, 0.0;

    // 2. Instantiate and Setup Controller Classes
    DenseFormulation dense(horizon, nx, nu);
    dense.setSystemMatrices(A, B);
    dense.setStateLimits(x_min, x_max);
    dense.setInputLimits(u_min, u_max);
    dense.setCostMatrices(Q, R);
    dense.setup();

    SparseFormulation sparse(horizon, nx, nu);
    sparse.setSystemMatrices(A, B);
    sparse.setStateLimits(x_min, x_max);
    sparse.setInputLimits(u_min, u_max);
    sparse.setCostMatrices(Q, R);
    sparse.setup();

    // 3. Construct Stacked Weight Matrices for Explicit Evaluation
    Eigen::MatrixXd Q_full = Eigen::MatrixXd::Zero(horizon * nx, horizon * nx);
    Eigen::MatrixXd R_full = Eigen::MatrixXd::Zero(horizon * nu, horizon * nu);
    Eigen::VectorXd X_ref = Eigen::VectorXd::Zero(horizon * nx);

    for (int k = 0; k < horizon; ++k) {
        Q_full.block(k * nx, k * nx, nx, nx) = Q;
        R_full.block(k * nu, k * nu, nu, nu) = R;
        X_ref.segment(k * nx, nx) = x_ref;
    }

    // 4. Create a Single Test Control Trajectory Vector U (u0 to u_{N-1})
    Eigen::VectorXd U(horizon * nu);
    for (int k = 0; k < horizon; ++k) {
        U(2 * k) = 150.0 - 10.0 * k;
        U(2 * k + 1) = -120.0 + 8.0 * k;
    }

    // Compute Exact Predicted States X (x1 to x_N) using discrete dynamics
    Eigen::VectorXd X(horizon * nx);
    Eigen::VectorXd curr_x = x0;
    for (int k = 0; k < horizon; ++k) {
        Eigen::VectorXd u_k = U.segment(k * nu, nu);
        curr_x = A * curr_x + B * u_k;
        X.segment(k * nx, nx) = curr_x;
    }

    // Construct Interleaved Decision Vector z = [u0, x1, u1, x2, ..., u_{N-1}, x_N]
    int n_step = nu + nx;
    int n_var = horizon * n_step;
    Eigen::VectorXd z(n_var);
    for (int k = 0; k < horizon; ++k) {
        z.segment(k * n_step, nu) = U.segment(k * nu, nu);
        z.segment(k * n_step + nu, nx) = X.segment(k * nx, nx);
    }

    // 5. Evaluate Physical Cost Function J = sum_{k=1}^N (x_k - x_ref)' Q (x_k - x_ref) + sum_{k=0}^{N-1} u_k' R u_k
    double J_physical = 0.0;
    for (int k = 0; k < horizon; ++k) {
        Eigen::VectorXd u_k = U.segment(k * nu, nu);
        Eigen::VectorXd x_k = X.segment(k * nx, nx); // x_{k+1}
        Eigen::VectorXd x_err = x_k - x_ref;

        J_physical += x_err.transpose() * Q * x_err;
        J_physical += u_k.transpose() * R * u_k;
    }

    // 6. Evaluate Dense QP Matrix Cost (with offset)
    Eigen::MatrixXd Sx(horizon * nx, nx);
    Eigen::MatrixXd Su = Eigen::MatrixXd::Zero(horizon * nx, horizon * nu);

    Eigen::MatrixXd A_pow = A;
    for (int i = 0; i < horizon; ++i) {
        Sx.block(i * nx, 0, nx, nx) = A_pow;
        for (int j = 0; j <= i; ++j) {
            Eigen::MatrixXd A_diff = Eigen::MatrixXd::Identity(nx, nx);
            for (int p = 0; p < (i - j); ++p) {
                A_diff = A_diff * A;
            }
            Su.block(i * nx, j * nu, nx, nu) = A_diff * B;
        }
        A_pow = A_pow * A;
    }

    Eigen::MatrixXd H_dense = 2.0 * (Su.transpose() * Q_full * Su + R_full);
    Eigen::VectorXd g_dense = 2.0 * Su.transpose() * Q_full * (Sx * x0 - X_ref);
    double const_dense = (Sx * x0 - X_ref).transpose() * Q_full * (Sx * x0 - X_ref);

    double J_dense_QP = 0.5 * U.transpose() * H_dense * U + g_dense.dot(U) + const_dense;

    // 7. Evaluate Sparse QP Matrix Cost (with offset)
    Eigen::MatrixXd Sx_sp = Eigen::MatrixXd::Zero(horizon * nx, n_var);
    Eigen::MatrixXd Su_sp = Eigen::MatrixXd::Zero(horizon * nu, n_var);
    for (int k = 0; k < horizon; ++k) {
        Sx_sp.block(k * nx, k * n_step + nu, nx, nx) = Eigen::MatrixXd::Identity(nx, nx);
        Su_sp.block(k * nu, k * n_step, nu, nu) = Eigen::MatrixXd::Identity(nu, nu);
    }

    Eigen::MatrixXd H_sparse = 2.0 * (Sx_sp.transpose() * Q_full * Sx_sp + Su_sp.transpose() * R_full * Su_sp);
    Eigen::VectorXd g_sparse = -2.0 * Sx_sp.transpose() * Q_full * X_ref;
    double const_sparse = X_ref.transpose() * Q_full * X_ref;

    double J_sparse_QP = 0.5 * z.transpose() * H_sparse * z + g_sparse.dot(z) + const_sparse;

    std::cout << std::fixed << std::setprecision(12);
    std::cout << "==========================================================================================================" << std::endl;
    std::cout << "EVALUATING THE EXACT SAME TRAJECTORY VECTOR [U, X] ACROSS ALL COST FORMULAS" << std::endl;
    std::cout << "==========================================================================================================" << std::endl;
    std::cout << "1. Physical Cost J_physical(U, X):          " << J_physical << std::endl;
    std::cout << "2. Dense QP Cost J_dense_QP(U) + offset:    " << J_dense_QP << std::endl;
    std::cout << "3. Sparse QP Cost J_sparse_QP(z) + offset:  " << J_sparse_QP << std::endl;
    std::cout << "----------------------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "Dense vs Physical Difference:              " << std::abs(J_dense_QP - J_physical) << std::endl;
    std::cout << "Sparse vs Physical Difference:             " << std::abs(J_sparse_QP - J_physical) << std::endl;
    std::cout << "Dense vs Sparse Difference:               " << std::abs(J_dense_QP - J_sparse_QP) << std::endl;
    std::cout << "==========================================================================================================" << std::endl;

    return 0;
}
