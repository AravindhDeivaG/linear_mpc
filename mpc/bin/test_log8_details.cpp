#include "mpc_controller.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <Eigen/Dense>

void printSolverDetails(const std::string& name, MpcController& mpc) {
    std::cout << "==========================================================================================================" << std::endl;
    std::cout << "SOLVER DETAILS FOR LOG #8: " << name << std::endl;
    std::cout << "==========================================================================================================" << std::endl;
    std::cout << "solution->info->status:   " << mpc.getStatusString() << " (Code: " << mpc.getStatus() << ")" << std::endl;
    std::cout << "solution->info->iter:     " << mpc.getIterations() << std::endl;
    std::cout << std::fixed << std::setprecision(8);
    std::cout << "solution->info->obj_val:  " << mpc.getObjectiveValue() << std::endl;
    std::cout << std::scientific << std::setprecision(6);
    std::cout << "solution->info->prim_res: " << mpc.getPrimalResidual() << std::endl;
    std::cout << "solution->info->dual_res: " << mpc.getDualResidual() << std::endl;
    std::cout << "----------------------------------------------------------------------------------------------------------" << std::endl;

    Eigen::VectorXd sol = mpc.getRawSolution();
    std::cout << "solution->x (dimension = " << sol.size() << "):" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    for (int i = 0; i < sol.size(); ++i) {
        std::cout << "  x[" << std::setw(2) << i << "] = " << std::setw(12) << sol(i);
        if ((i + 1) % 4 == 0) std::cout << "\n";
    }
    if (sol.size() % 4 != 0) std::cout << "\n";
    std::cout << "==========================================================================================================" << std::endl;
}

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

    // Log #8 state & reference
    Eigen::VectorXd x(nx);
    x << 219.599984, 280.400015, 139.999937, -139.999937;

    Eigen::VectorXd x_ref(nx);
    x_ref << 623.0, 80.0, 0.0, 0.0;

    // 1. Dense MPC
    MpcController dense_mpc(horizon, nx, nu, FormulationType::DENSE);
    dense_mpc.setSystemMatrices(A, B);
    dense_mpc.setStateLimits(x_min, x_max);
    dense_mpc.setInputLimits(u_min, u_max);
    dense_mpc.setCostMatrices(Q, R);
    dense_mpc.setup();
    dense_mpc.setCurrentState(x);
    dense_mpc.setReferenceState(x_ref);
    dense_mpc.doControl();

    // 2. Sparse MPC
    MpcController sparse_mpc(horizon, nx, nu, FormulationType::SPARSE);
    sparse_mpc.setSystemMatrices(A, B);
    sparse_mpc.setStateLimits(x_min, x_max);
    sparse_mpc.setInputLimits(u_min, u_max);
    sparse_mpc.setCostMatrices(Q, R);
    sparse_mpc.setup();
    sparse_mpc.setCurrentState(x);
    sparse_mpc.setReferenceState(x_ref);
    sparse_mpc.doControl();

    // Print details
    printSolverDetails("DENSE MPC FORMULATION", dense_mpc);
    std::cout << "\n";
    printSolverDetails("SPARSE MPC FORMULATION", sparse_mpc);

    return 0;
}
