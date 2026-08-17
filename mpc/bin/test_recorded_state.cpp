#include "mpc_controller.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <Eigen/Dense>

int main() {
    const int horizon = 10;
    const int nx = 4;
    const int nu = 2;
    const double dt = 0.02;

    Eigen::VectorXd x(nx);
    x << 200.0, 300.0, 0.0, 0.0;

    Eigen::VectorXd x_ref(nx);
    x_ref << 400.0, 300.0, 0.0, 0.0;

    Eigen::VectorXd u_dense_rec(nu);
    u_dense_rec.setZero();
    Eigen::VectorXd u_sparse_rec(nu);
    u_sparse_rec.setZero();

    // Try reading recorded_state.txt
    std::ifstream ifs("recorded_state.txt");
    if (ifs.is_open()) {
        ifs >> x(0) >> x(1) >> x(2) >> x(3);
        ifs >> x_ref(0) >> x_ref(1) >> x_ref(2) >> x_ref(3);
        ifs >> u_dense_rec(0) >> u_dense_rec(1);
        ifs >> u_sparse_rec(0) >> u_sparse_rec(1);
        ifs.close();
        std::cout << "[SUCCESS] Loaded snapshot from recorded_state.txt" << std::endl;
    } else {
        std::cout << "[INFO] recorded_state.txt not found. Using default test state." << std::endl;
    }

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\n==========================================" << std::endl;
    std::cout << "NUMERICAL DIAGNOSTIC FOR RECORDED STATE" << std::endl;
    std::cout << "==========================================" << std::endl;
    std::cout << "State x:     [" << x.transpose() << "]" << std::endl;
    std::cout << "State x_ref: [" << x_ref.transpose() << "]" << std::endl;

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

    // 1. Solve Dense
    MpcController dense_mpc(horizon, nx, nu, FormulationType::DENSE);
    dense_mpc.setSystemMatrices(A, B);
    dense_mpc.setStateLimits(x_min, x_max);
    dense_mpc.setInputLimits(u_min, u_max);
    dense_mpc.setCostMatrices(Q, R);
    dense_mpc.setup();

    dense_mpc.setCurrentState(x);
    dense_mpc.setReferenceState(x_ref);
    dense_mpc.doControl();

    Eigen::VectorXd u_dense, X_dense, U_dense;
    dense_mpc.getOptimalControl(u_dense);
    dense_mpc.getPredictedStates(X_dense);
    dense_mpc.getPredictedInputs(U_dense);

    // 2. Solve Sparse
    MpcController sparse_mpc(horizon, nx, nu, FormulationType::SPARSE);
    sparse_mpc.setSystemMatrices(A, B);
    sparse_mpc.setStateLimits(x_min, x_max);
    sparse_mpc.setInputLimits(u_min, u_max);
    sparse_mpc.setCostMatrices(Q, R);
    sparse_mpc.setup();

    sparse_mpc.setCurrentState(x);
    sparse_mpc.setReferenceState(x_ref);
    sparse_mpc.doControl();

    Eigen::VectorXd u_sparse, X_sparse, U_sparse;
    sparse_mpc.getOptimalControl(u_sparse);
    sparse_mpc.getPredictedStates(X_sparse);
    sparse_mpc.getPredictedInputs(U_sparse);

    std::cout << "\n==========================================" << std::endl;
    std::cout << "SOLVER FIRST STEP INPUT u0:" << std::endl;
    std::cout << "==========================================" << std::endl;
    std::cout << "Dense  u0: (" << u_dense(0) << ", " << u_dense(1) << ")" << std::endl;
    std::cout << "Sparse u0: (" << u_sparse(0) << ", " << u_sparse(1) << ")" << std::endl;
    std::cout << "u0 Norm Diff: " << (u_dense - u_sparse).norm() << std::endl;

    std::cout << "\n==========================================" << std::endl;
    std::cout << "PREDICTED CONTROL INPUTS OVER HORIZON (U_dense vs U_sparse):" << std::endl;
    std::cout << "==========================================" << std::endl;
    for (int k = 0; k < horizon; ++k) {
        double du0 = U_dense(2*k) - U_sparse(2*k);
        double du1 = U_dense(2*k+1) - U_sparse(2*k+1);
        std::cout << "Step k=" << std::setw(2) << k 
                  << " | Dense u: (" << std::setw(10) << U_dense(2*k) << ", " << std::setw(10) << U_dense(2*k+1) << ")"
                  << " | Sparse u: (" << std::setw(10) << U_sparse(2*k) << ", " << std::setw(10) << U_sparse(2*k+1) << ")"
                  << " | Diff: (" << std::setw(10) << du0 << ", " << std::setw(10) << du1 << ")" << std::endl;
    }

    std::cout << "\n==========================================" << std::endl;
    std::cout << "PREDICTED STATES OVER HORIZON (X_dense vs X_sparse):" << std::endl;
    std::cout << "==========================================" << std::endl;
    for (int k = 0; k < horizon; ++k) {
        double dx_pos = X_dense(4*k) - X_sparse(4*k);
        double dy_pos = X_dense(4*k+1) - X_sparse(4*k+1);
        std::cout << "Step k=" << std::setw(2) << k + 1 
                  << " | Dense X: (" << std::setw(9) << X_dense(4*k) << ", " << std::setw(9) << X_dense(4*k+1) << ")"
                  << " | Sparse X: (" << std::setw(9) << X_sparse(4*k) << ", " << std::setw(9) << X_sparse(4*k+1) << ")"
                  << " | Diff: (" << std::setw(9) << dx_pos << ", " << std::setw(9) << dy_pos << ")" << std::endl;
    }

    return 0;
}
