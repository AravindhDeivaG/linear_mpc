#include "mpc_controller.h"
#include <iostream>
#include <Eigen/Dense>

int main() {
    std::cout << "Starting MPC Validation Test..." << std::endl;

    // 1. Initialize MPC Controller with horizon 1 and dt 0.1
    int horizon = 5;
    double dt = 0.1;
    MpcController controller(horizon, dt);

    // 2. Create and set initial state x (size 4)
    Eigen::VectorXd x(4);
    x << 0.0, 0.0, 0.0, 0.0;
    controller.setCurrentState(x);

    // 3. Create and set reference state x_ref (size 4)
    Eigen::VectorXd x_ref(4);
    x_ref << 10.0, 10.0, 0.0, 0.0;
    controller.setReferenceState(x_ref);

    // 4. Run one loop of control logic to verify matrix products and solver
    std::cout << "Running doControl()..." << std::endl;
    controller.doControl();

    Eigen::VectorXd u_opt;
    controller.getOptimalControl(u_opt);
    std::cout << "Optimal control u_opt: \n" << u_opt << std::endl;

    controller.doControl();
    controller.getOptimalControl(u_opt);
    std::cout << "Optimal control u_opt: \n" << u_opt << std::endl;


    std::cout << "MPC Validation Test completed successfully!" << std::endl;
    return 0;
}
