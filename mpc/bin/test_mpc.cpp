#include "mpc_controller.h"
#include <iostream>
#include <Eigen/Dense>

void testFormulation(FormulationType type, const std::string& name) {
    std::cout << "\n==========================================" << std::endl;
    std::cout << "Testing " << name << " Formulation" << std::endl;
    std::cout << "==========================================" << std::endl;

    int horizon = 10;
    int nx = 4;
    int nu = 2;
    double dt = 0.1;

    // Define 2D double-integrator system matrices
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
    R.setZero();

    // 1. Initialize MPC Controller with horizon, nx, nu, dt, and formulation type
    MpcController controller(horizon, nx, nu, type);
    controller.setSystemMatrices(A, B);
    controller.setStateLimits(x_min, x_max);
    controller.setInputLimits(u_min, u_max);
    controller.setCostMatrices(Q, R);
    controller.setup();

    // 2. Set initial state and reference state
    Eigen::VectorXd x(nx);
    x << 0.0, 0.0, 0.0, 0.0;
    controller.setCurrentState(x);

    Eigen::VectorXd x_ref(nx);
    x_ref << 10.0, 10.0, 0.0, 0.0;
    controller.setReferenceState(x_ref);

    // 3. Run control loops
    controller.doControl();
    Eigen::VectorXd u_opt;
    controller.getOptimalControl(u_opt);
    std::cout << "Optimal control u_opt (Step 1): \n" << u_opt.transpose() << std::endl;

    Eigen::VectorXd X_pred;
    controller.getPredictedStates(X_pred);
    std::cout << "Predicted X position (Horizon Step 1): (" << X_pred(0) << ", " << X_pred(1) << ")" << std::endl;
}

int main() {
    std::cout << "Starting MPC Formulation Comparison Test..." << std::endl;

    testFormulation(FormulationType::DENSE, "DENSE");
    testFormulation(FormulationType::SPARSE, "SPARSE");

    std::cout << "\nMPC Comparison Test completed successfully!" << std::endl;
    return 0;
}
