#include "mpc_controller.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <cmath>
#include <vector>
#include <Eigen/Dense>

int main(int argc, char** argv) {
    const int horizon = 10;
    const int nx = 4;
    const int nu = 2;
    const double dt = 0.02;

    Eigen::VectorXd x_target(nx);
    x_target << 491.985093, 466.758085, 199.090784, -4.436237;

    Eigen::VectorXd x_ref(nx);
    x_ref << 525.000000, 467.000000, 0.000000, 0.000000;

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

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\n==========================================================================================================" << std::endl;
    std::cout << "COLD SOLVE (ONE SHOT) VS WARM-STARTED SOLVE COMPARISON FOR TARGET STATE" << std::endl;
    std::cout << "Target State x: [" << x_target.transpose() << "]" << std::endl;
    std::cout << "==========================================================================================================" << std::endl;

    // 1. Dense MPC Reference
    MpcController dense_mpc(horizon, nx, nu, FormulationType::DENSE);
    dense_mpc.setSystemMatrices(A, B);
    dense_mpc.setStateLimits(x_min, x_max);
    dense_mpc.setInputLimits(u_min, u_max);
    dense_mpc.setCostMatrices(Q, R);
    dense_mpc.setup();

    dense_mpc.setCurrentState(x_target);
    dense_mpc.setReferenceState(x_ref);
    dense_mpc.doControl();

    Eigen::VectorXd u_dense;
    dense_mpc.getOptimalControl(u_dense);

    std::cout << "Dense MPC Reference u0:              (" << std::setw(10) << u_dense(0) << ", " << std::setw(10) << u_dense(1) << ")" << std::endl;
    std::cout << "----------------------------------------------------------------------------------------------------------" << std::endl;

    // 2. Sparse MPC: ONE SHOT (COLD SOLVE)
    MpcController sparse_cold(horizon, nx, nu, FormulationType::SPARSE);
    sparse_cold.setSystemMatrices(A, B);
    sparse_cold.setStateLimits(x_min, x_max);
    sparse_cold.setInputLimits(u_min, u_max);
    sparse_cold.setCostMatrices(Q, R);
    sparse_cold.setup();

    sparse_cold.setCurrentState(x_target);
    sparse_cold.setReferenceState(x_ref);
    sparse_cold.doControl();

    Eigen::VectorXd u_sparse_cold;
    sparse_cold.getOptimalControl(u_sparse_cold);

    std::cout << "Sparse MPC - ONE SHOT (Cold Solve) u0: (" << std::setw(10) << u_sparse_cold(0) << ", " << std::setw(10) << u_sparse_cold(1) 
              << ") | Iter: " << std::setw(3) << sparse_cold.getIterations()
              << " | Diff vs Dense: " << (u_dense - u_sparse_cold).norm() << std::endl;

    // 3. Sparse MPC: WARM-STARTED CHAIN SOLVE
    MpcController sparse_warm(horizon, nx, nu, FormulationType::SPARSE);
    sparse_warm.setSystemMatrices(A, B);
    sparse_warm.setStateLimits(x_min, x_max);
    sparse_warm.setInputLimits(u_min, u_max);
    sparse_warm.setCostMatrices(Q, R);
    sparse_warm.setup();

    // Sequence of previous states
    Eigen::VectorXd s1(nx), s2(nx), s3(nx);
    s1 << 471.999854, 470.983781, 200.003954, -35.571769;
    s2 << 475.999929, 470.285123, 200.003546, -34.294094;
    s3 << 487.996425, 466.877997, 199.775954, -7.554958;

    sparse_warm.setCurrentState(s1); sparse_warm.setReferenceState(x_ref); sparse_warm.doControl();
    sparse_warm.setCurrentState(s2); sparse_warm.setReferenceState(x_ref); sparse_warm.doControl();
    sparse_warm.setCurrentState(s3); sparse_warm.setReferenceState(x_ref); sparse_warm.doControl();

    // Now solve target state with warm start buffer
    sparse_warm.setCurrentState(x_target);
    sparse_warm.setReferenceState(x_ref);
    sparse_warm.doControl();

    Eigen::VectorXd u_sparse_warm;
    sparse_warm.getOptimalControl(u_sparse_warm);

    std::cout << "Sparse MPC - WARM-STARTED Solve u0:    (" << std::setw(10) << u_sparse_warm(0) << ", " << std::setw(10) << u_sparse_warm(1) 
              << ") | Iter: " << std::setw(3) << sparse_warm.getIterations()
              << " | Diff vs Dense: " << (u_dense - u_sparse_warm).norm() << std::endl;

    std::cout << "==========================================================================================================" << std::endl;

    return 0;
}
