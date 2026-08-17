#include "mpc_controller.h"
#include <iostream>
#include <Eigen/Dense>

void testConvergence(FormulationType type, const std::string& name) {
    std::cout << "\n==========================================" << std::endl;
    std::cout << "Testing Convergence: " << name << " Formulation" << std::endl;
    std::cout << "==========================================" << std::endl;

    int horizon = 20;
    int nx = 4;
    int nu = 2;
    double dt = 0.02;

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

    MpcController controller(horizon, nx, nu, type);
    controller.setSystemMatrices(A, B);
    controller.setStateLimits(x_min, x_max);
    controller.setInputLimits(u_min, u_max);
    controller.setCostMatrices(Q, R);
    controller.setup();

    Eigen::VectorXd x(nx);
    x << 200.0, 300.0, 0.0, 0.0;

    Eigen::VectorXd x_ref(nx);
    x_ref << 400.0, 300.0, 0.0, 0.0;

    controller.setReferenceState(x_ref);

    for (int step = 0; step < 15; ++step) {
        controller.setCurrentState(x);
        controller.doControl();
        
        Eigen::VectorXd u;
        controller.getOptimalControl(u);

        x(0) += x(2)*dt + 0.5*u(0)*dt*dt;
        x(1) += x(3)*dt + 0.5*u(1)*dt*dt;
        x(2) += u(0)*dt;
        x(3) += u(1)*dt;

        std::cout << "Step " << step + 1 << ": Pos = (" << x(0) << ", " << x(1) 
                  << ") | u = (" << u(0) << ", " << u(1) << ")" << std::endl;
    }
}

int main() {
    testConvergence(FormulationType::DENSE, "DENSE");
    testConvergence(FormulationType::SPARSE, "SPARSE");
    return 0;
}
