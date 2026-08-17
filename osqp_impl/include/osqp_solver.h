#ifndef OSQP_SOLVER_H
#define OSQP_SOLVER_H

#include <Eigen/Dense>
#include <Eigen/Sparse>

class OsqpSolver {
public:
    // Constructor: sets the dimensions of variables (n) and constraints (m)
    OsqpSolver(int n, int m);

    // Destructor: cleans up OSQP solver workspace
    ~OsqpSolver();

    // Set the quadratic objective matrix P (dense size n x n)
    void setHessian(const Eigen::MatrixXd& P);

    // Set the linear objective vector q (size n)
    void setGradient(const Eigen::VectorXd& q);

    // Set the linear constraint matrix A (dense size m x n)
    void setConstraintMatrix(const Eigen::MatrixXd& A);

    // Set the lower bound constraint vector l (size m)
    void setLowerBound(const Eigen::VectorXd& l);

    // Set the upper bound constraint vector u (size m)
    void setUpperBound(const Eigen::VectorXd& u);

    // Solve the Quadratic Program
    // Automatically sets up the workspace on the first run, and runs fast updates on subsequent runs.
    // Returns true if solved successfully (optimal or solved inaccurate).
    bool solve();

    // Get the primal solution vector x (size n)
    Eigen::VectorXd getSolution() const;

    // Solver diagnostic queries
    int getStatus() const;
    int getIterations() const;
    double getPrimalResidual() const;
    double getDualResidual() const;

private:
    int m_n; // Number of variables
    int m_m; // Number of constraints

    void* m_solver; // void* to avoid exposing OSQP C structs in public headers
    bool m_is_initialized;
    bool m_matrices_need_update;

    // Cache the dense matrices and vectors
    Eigen::MatrixXd m_P;
    Eigen::VectorXd m_q;
    Eigen::MatrixXd m_A;
    Eigen::VectorXd m_l;
    Eigen::VectorXd m_u;

    // Sparse matrix structures for OSQP (we keep their memory buffers valid).
    // The storage index type (third template argument) is long long (64-bit) to match OSQPInt on this platform.
    Eigen::SparseMatrix<double, Eigen::ColMajor, long long> m_P_sparse;
    Eigen::SparseMatrix<double, Eigen::ColMajor, long long> m_A_sparse;
};

#endif // OSQP_SOLVER_H
