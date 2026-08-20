#include "mpc_controller.h"
#include "osqp_solver.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <Eigen/Dense>

int main() {
    const int horizon = 15;
    const int nx = 4;
    const int nu = 2;
    const double dt = 0.02;

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

    std::cout << "==========================================================================================================" << std::endl;
    std::cout << "EXAMINING LOG #8 WITH ACTIVE VELOCITY BOUNDS (v_max = 200)" << std::endl;
    std::cout << "==========================================================================================================" << std::endl;

    MpcController dense_mpc(horizon, nx, nu, FormulationType::DENSE);
    dense_mpc.setSystemMatrices(A, B);
    dense_mpc.setStateLimits(x_min, x_max);
    dense_mpc.setInputLimits(u_min, u_max);
    dense_mpc.setCostMatrices(Q, R);
    dense_mpc.setup();

    MpcController sparse_mpc(horizon, nx, nu, FormulationType::SPARSE);
    sparse_mpc.setSystemMatrices(A, B);
    sparse_mpc.setStateLimits(x_min, x_max);
    sparse_mpc.setInputLimits(u_min, u_max);
    sparse_mpc.setCostMatrices(Q, R);
    sparse_mpc.setup();

    Eigen::VectorXd x(nx);
    x << 219.599984, 280.400015, 139.999937, -139.999937;

    Eigen::VectorXd x_ref(nx);
    x_ref << 623.0, 80.0, 0.0, 0.0;

    std::cout << "Step | State (x, y, vx, vy)                       | Dense (u0, iter)      | Sparse (u0, iter)     | Diff" << std::endl;
    std::cout << "-----+-------------------------------------------+-----------------------+-----------------------+---------" << std::endl;

    for (int step = 0; step < 7; ++step) {
        dense_mpc.setCurrentState(x);
        dense_mpc.setReferenceState(x_ref);
        dense_mpc.doControl();
        Eigen::VectorXd u_dense;
        dense_mpc.getOptimalControl(u_dense);
        int dense_iters = dense_mpc.getIterations();

        sparse_mpc.setCurrentState(x);
        sparse_mpc.setReferenceState(x_ref);
        sparse_mpc.doControl();
        Eigen::VectorXd u_sparse;
        sparse_mpc.getOptimalControl(u_sparse);
        int sparse_iters = sparse_mpc.getIterations();

        double diff_u = (u_dense - u_sparse).norm();

        std::cout << std::setw(4) << step + 1 << " | ("
                  << std::setw(8) << std::fixed << std::setprecision(1) << x(0) << ", " << std::setw(8) << x(1) << ", "
                  << std::setw(7) << x(2) << ", " << std::setw(7) << x(3) << ") | ("
                  << std::setw(7) << std::setprecision(3) << u_dense(0) << ", " << std::setw(7) << u_dense(1) << ", " << std::setw(4) << dense_iters << ") | ("
                  << std::setw(7) << u_sparse(0) << ", " << std::setw(7) << u_sparse(1) << ", " << std::setw(4) << sparse_iters << ") | "
                  << std::setw(7) << std::setprecision(4) << diff_u << std::endl;

        x(0) += x(2)*dt + 0.5*u_sparse(0)*dt*dt;
        x(1) += x(3)*dt + 0.5*u_sparse(1)*dt*dt;
        x(2) += u_sparse(0)*dt;
        x(3) += u_sparse(1)*dt;
    }

    return 0;
}
