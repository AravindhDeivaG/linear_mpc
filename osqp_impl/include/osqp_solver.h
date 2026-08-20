#ifndef OSQP_SOLVER_H
#define OSQP_SOLVER_H

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <string>

struct OsqpConfig {
    bool verbose = false;
    bool enable_timing = false;
    double time_threshold_ms = 1.0;
    int check_termination = 1;
    double rho = 5.0;
    bool adaptive_rho = false;
    double eps_abs = 1e-3;
    double eps_rel = 1e-3;
    int max_iter = 4000;
    bool polishing = false;

    bool loadFromYaml(const std::string& filepath);
};

class OsqpSolver {
public:
    // Constructor: sets the dimensions of variables (n) and constraints (m)
    OsqpSolver(int n, int m);

    // Destructor: cleans up OSQP solver workspace
    ~OsqpSolver();

    // Configuration & logging options
    void setConfig(const OsqpConfig& config);
    bool loadConfig(const std::string& yaml_path);
    OsqpConfig getConfig() const;
    void setVerbose(bool verbose);
    void setLogTiming(bool enable, double threshold_ms = 1.0);

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

    // Set initial guess for primal and dual variables (Warm Start)
    void setWarmStart(const Eigen::VectorXd& x);
    void setWarmStartDual(const Eigen::VectorXd& y);

    // Solve the Quadratic Program
    // Automatically sets up the workspace on the first run, and runs fast updates on subsequent runs.
    // Returns true if solved successfully (optimal or solved inaccurate).
    bool solve();

    // Get the primal solution vector x (size n)
    Eigen::VectorXd getSolution() const;

    // Get the dual solution vector y (size m)
    Eigen::VectorXd getDualSolution() const;

    // Solver diagnostic queries
    int getStatus() const;
    const char* getStatusString() const;
    int getIterations() const;
    double getObjectiveValue() const;
    double getPrimalResidual() const;
    double getDualResidual() const;

private:
    int m_n; // Number of variables
    int m_m; // Number of constraints

    OsqpConfig m_config;
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
