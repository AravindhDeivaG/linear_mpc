#include "osqp_solver.h"
#include <iostream>
#include <iomanip>

int main() {
    // We have 2 variables (x, y) and 2 dummy constraints
    OsqpSolver solver(2, 2);

    // Quadratic term P = [9, 3; 3, 4]
    Eigen::MatrixXd P(2, 2);
    P << 9.0, 3.0,
         3.0, 4.0;

    // Linear term g = [-12, -8]
    Eigen::VectorXd q(2);
    q << -12.0, -8.0;

    // Constraint matrix A = [0, 0; 0, 0] (no active constraints)
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2, 2);

    // Bounds
    Eigen::VectorXd l(2);
    l << -1000.0, -1000.0;

    Eigen::VectorXd u(2);
    u << 1000.0, 1000.0;

    // Set matrices and vectors in solver
    solver.setHessian(P);
    solver.setGradient(q);
    solver.setConstraintMatrix(A);
    solver.setLowerBound(l);
    solver.setUpperBound(u);

    // Solve the Quadratic Program
    if (solver.solve()) {
        Eigen::VectorXd solution = solver.getSolution();
        std::cout << "QP solved successfully!\n";
        std::cout << std::fixed << std::setprecision(4);
        std::cout << "Optimal Solution:\n";
        std::cout << "x = " << solution(0) << " (Analytical: 0.8889)\n";
        std::cout << "y = " << solution(1) << " (Analytical: 1.3333)\n";
    } else {
        std::cerr << "QP solver failed to find a solution.\n";
        return -1;
    }

    return 0;
}
