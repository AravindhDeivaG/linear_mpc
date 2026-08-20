#include "mpc_controller.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <Eigen/Dense>

int main() {
    const int horizon = 10;
    const int nx = 2; // 1D system: [position, velocity]
    const int nu = 1; // 1D control:  [acceleration]
    const double dt = 0.02;

    // 1D Double Integrator System Dynamics
    Eigen::MatrixXd A(nx, nx);
    A << 1.0,  dt,
         0.0, 1.0;

    Eigen::MatrixXd B(nx, nu);
    B << 0.5 * dt * dt,
         dt;

    Eigen::VectorXd x_min(nx), x_max(nx);
    x_min << -10000.0, -200.0;
    x_max <<  10000.0,  200.0;

    Eigen::VectorXd u_min(nu), u_max(nu);
    u_min << -500.0;
    u_max <<  500.0;

    Eigen::MatrixXd Q(nx, nx);
    Q << 10.0, 0.0,
          0.0, 0.1;
    Q *= 1000.0;

    Eigen::MatrixXd R(nu, nu);
    R << 1e-3;
    R *= 1000.0;

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\n==========================================================================================================" << std::endl;
    std::cout << "1D TOY MPC EXPERIMENT: INDEPENDENT LONGITUDINAL (X) AND LATERAL (Y) SOLVES" << std::endl;
    std::cout << "==========================================================================================================" << std::endl;

    // ------------------------------------------------------------------------------------------------------------------
    // TEST 1: 1D LONGITUDINAL (X-AXIS)
    // ------------------------------------------------------------------------------------------------------------------
    Eigen::VectorXd x0_x(nx);
    x0_x << 491.985093, 199.090784; // pos_x, vel_x (near 200 m/s limit)

    Eigen::VectorXd xref_x(nx);
    xref_x << 525.000000, 0.000000; // target pos_x, target vel_x

    // Dense 1D X
    MpcController dense_1d_x(horizon, nx, nu, FormulationType::DENSE);
    dense_1d_x.setSystemMatrices(A, B);
    dense_1d_x.setStateLimits(x_min, x_max);
    dense_1d_x.setInputLimits(u_min, u_max);
    dense_1d_x.setCostMatrices(Q, R);
    dense_1d_x.setup();
    dense_1d_x.setCurrentState(x0_x);
    dense_1d_x.setReferenceState(xref_x);
    dense_1d_x.doControl();

    Eigen::VectorXd u_dense_x;
    dense_1d_x.getOptimalControl(u_dense_x);

    // Sparse 1D X (Cold)
    MpcController sparse_1d_x(horizon, nx, nu, FormulationType::SPARSE);
    sparse_1d_x.setSystemMatrices(A, B);
    sparse_1d_x.setStateLimits(x_min, x_max);
    sparse_1d_x.setInputLimits(u_min, u_max);
    sparse_1d_x.setCostMatrices(Q, R);
    sparse_1d_x.setup();
    sparse_1d_x.setCurrentState(x0_x);
    sparse_1d_x.setReferenceState(xref_x);
    sparse_1d_x.doControl();

    Eigen::VectorXd u_sparse_x;
    sparse_1d_x.getOptimalControl(u_sparse_x);

    std::cout << "\n--- TEST 1: 1D LONGITUDINAL (X-AXIS) ---" << std::endl;
    std::cout << "Initial State: [pos_x = " << x0_x(0) << ", vel_x = " << x0_x(1) << "]" << std::endl;
    std::cout << "Target State:  [pos_x = " << xref_x(0) << ", vel_x = " << xref_x(1) << "]" << std::endl;
    std::cout << "Dense  1D ux0: " << std::setw(11) << u_dense_x(0)  << " | Status: " << dense_1d_x.getStatusString()  << " | Iter: " << dense_1d_x.getIterations() << std::endl;
    std::cout << "Sparse 1D ux0: " << std::setw(11) << u_sparse_x(0) << " | Status: " << sparse_1d_x.getStatusString() << " | Iter: " << sparse_1d_x.getIterations() << std::endl;
    std::cout << "Abs Diff |ux0_dense - ux0_sparse|: " << std::abs(u_dense_x(0) - u_sparse_x(0)) << std::endl;

    // ------------------------------------------------------------------------------------------------------------------
    // TEST 2: 1D LATERAL (Y-AXIS)
    // ------------------------------------------------------------------------------------------------------------------
    Eigen::VectorXd x0_y(nx);
    x0_y << 466.758085, -4.436237; // pos_y, vel_y

    Eigen::VectorXd xref_y(nx);
    xref_y << 467.000000, 0.000000; // target pos_y, target vel_y

    // Dense 1D Y
    MpcController dense_1d_y(horizon, nx, nu, FormulationType::DENSE);
    dense_1d_y.setSystemMatrices(A, B);
    dense_1d_y.setStateLimits(x_min, x_max);
    dense_1d_y.setInputLimits(u_min, u_max);
    dense_1d_y.setCostMatrices(Q, R);
    dense_1d_y.setup();
    dense_1d_y.setCurrentState(x0_y);
    dense_1d_y.setReferenceState(xref_y);
    dense_1d_y.doControl();

    Eigen::VectorXd u_dense_y;
    dense_1d_y.getOptimalControl(u_dense_y);

    // Sparse 1D Y (Cold)
    MpcController sparse_1d_y(horizon, nx, nu, FormulationType::SPARSE);
    sparse_1d_y.setSystemMatrices(A, B);
    sparse_1d_y.setStateLimits(x_min, x_max);
    sparse_1d_y.setInputLimits(u_min, u_max);
    sparse_1d_y.setCostMatrices(Q, R);
    sparse_1d_y.setup();
    sparse_1d_y.setCurrentState(x0_y);
    sparse_1d_y.setReferenceState(xref_y);
    sparse_1d_y.doControl();

    Eigen::VectorXd u_sparse_y;
    sparse_1d_y.getOptimalControl(u_sparse_y);

    std::cout << "\n--- TEST 2: 1D LATERAL (Y-AXIS) ---" << std::endl;
    std::cout << "Initial State: [pos_y = " << x0_y(0) << ", vel_y = " << x0_y(1) << "]" << std::endl;
    std::cout << "Target State:  [pos_y = " << xref_y(0) << ", vel_y = " << xref_y(1) << "]" << std::endl;
    std::cout << "Dense  1D uy0: " << std::setw(11) << u_dense_y(0)  << " | Status: " << dense_1d_y.getStatusString()  << " | Iter: " << dense_1d_y.getIterations() << std::endl;
    std::cout << "Sparse 1D uy0: " << std::setw(11) << u_sparse_y(0) << " | Status: " << sparse_1d_y.getStatusString() << " | Iter: " << sparse_1d_y.getIterations() << std::endl;
    std::cout << "Abs Diff |uy0_dense - uy0_sparse|: " << std::abs(u_dense_y(0) - u_sparse_y(0)) << std::endl;

    std::cout << "\n==========================================================================================================" << std::endl;

    return 0;
}
