#include "osqp_solver.h"
#include <osqp/osqp.h>
#include <iostream>
#include <stdexcept>

// Constructor
OsqpSolver::OsqpSolver(int n, int m)
    : m_n(n), m_m(m), m_solver(nullptr), m_is_initialized(false) {
    // Resize matrices and vectors to expected sizes
    m_P = Eigen::MatrixXd::Zero(m_n, m_n);
    m_q = Eigen::VectorXd::Zero(m_n);
    m_A = Eigen::MatrixXd::Zero(m_m, m_n);
    m_l = Eigen::VectorXd::Zero(m_m);
    m_u = Eigen::VectorXd::Zero(m_m);
}

// Destructor
OsqpSolver::~OsqpSolver() {
    if (m_solver) {
        osqp_cleanup(static_cast<OSQPSolver*>(m_solver));
    }
}

void OsqpSolver::setHessian(const Eigen::MatrixXd& P) {
    m_P = P;
}

void OsqpSolver::setGradient(const Eigen::VectorXd& q) {
    m_q = q;
}

void OsqpSolver::setConstraintMatrix(const Eigen::MatrixXd& A) {
    m_A = A;
}

void OsqpSolver::setLowerBound(const Eigen::VectorXd& l) {
    m_l = l;
}

void OsqpSolver::setUpperBound(const Eigen::VectorXd& u) {
    m_u = u;
}

bool OsqpSolver::solve() {
    OSQPSolver* solver = static_cast<OSQPSolver*>(m_solver);

    if (!m_is_initialized) {
        // 1. Dimension validation checks
        if (m_P.rows() != m_n || m_P.cols() != m_n) {
            throw std::runtime_error("Hessian P dimension mismatch: expected " + std::to_string(m_n) + "x" + std::to_string(m_n));
        }
        if (m_q.size() != m_n) {
            throw std::runtime_error("Gradient q dimension mismatch: expected size " + std::to_string(m_n));
        }
        if (m_A.rows() != m_m || m_A.cols() != m_n) {
            throw std::runtime_error("Constraint matrix A dimension mismatch: expected " + std::to_string(m_m) + "x" + std::to_string(m_n));
        }
        if (m_l.size() != m_m) {
            throw std::runtime_error("Lower bound l dimension mismatch: expected size " + std::to_string(m_m));
        }
        if (m_u.size() != m_m) {
            throw std::runtime_error("Upper bound u dimension mismatch: expected size " + std::to_string(m_m));
        }

        // 2. Pre-allocate sparse matrices with full structural non-zeros (column-major)
        // Hessian P is stored upper-triangular only for OSQP
        m_P_sparse.resize(m_n, m_n);
        m_P_sparse.reserve(Eigen::VectorXi::Constant(m_n, m_n));
        for (int col = 0; col < m_n; ++col) {
            for (int row = 0; row <= col; ++row) {
                m_P_sparse.insert(row, col) = m_P(row, col);
            }
        }
        m_P_sparse.makeCompressed();

        // Constraint matrix A is fully populated
        m_A_sparse.resize(m_m, m_n);
        m_A_sparse.reserve(Eigen::VectorXi::Constant(m_n, m_m));
        for (int col = 0; col < m_n; ++col) {
            for (int row = 0; row < m_m; ++row) {
                m_A_sparse.insert(row, col) = m_A(row, col);
            }
        }
        m_A_sparse.makeCompressed();

        // 3. Create OSQP CSC wrappers pointing directly to Eigen's memory buffers
        OSQPCscMatrix P_csc;
        OSQPCscMatrix_set_data(&P_csc, m_n, m_n, m_P_sparse.nonZeros(),
                               m_P_sparse.valuePtr(), 
                               reinterpret_cast<OSQPInt*>(m_P_sparse.innerIndexPtr()), 
                               reinterpret_cast<OSQPInt*>(m_P_sparse.outerIndexPtr()));

        OSQPCscMatrix A_csc;
        OSQPCscMatrix_set_data(&A_csc, m_m, m_n, m_A_sparse.nonZeros(),
                               m_A_sparse.valuePtr(), 
                               reinterpret_cast<OSQPInt*>(m_A_sparse.innerIndexPtr()), 
                               reinterpret_cast<OSQPInt*>(m_A_sparse.outerIndexPtr()));

        // 4. Setup default solver settings
        OSQPSettings settings;
        osqp_set_default_settings(&settings);
        settings.verbose = 0; // Disable verbose command output inside library

        // 5. Initialize the solver workspace
        OSQPSolver* solver_temp = nullptr;
        OSQPInt exitflag = osqp_setup(&solver_temp, &P_csc, m_q.data(), &A_csc,
                                      m_l.data(), m_u.data(), m_m, m_n, &settings);
        if (exitflag != 0) {
            std::cerr << "OSQP setup failed with code: " << exitflag << "\n";
            return false;
        }

        m_solver = solver_temp;
        solver = solver_temp;
        m_is_initialized = true;
    } else {
        // Subsequent fast updates:
        // Update sparse matrix value arrays (keeping the structure identical)
        int idx = 0;
        for (int col = 0; col < m_n; ++col) {
            for (int row = 0; row <= col; ++row) {
                m_P_sparse.valuePtr()[idx++] = m_P(row, col);
            }
        }

        idx = 0;
        for (int col = 0; col < m_n; ++col) {
            for (int row = 0; row < m_m; ++row) {
                m_A_sparse.valuePtr()[idx++] = m_A(row, col);
            }
        }

        // Update matrices in-place in OSQP workspace
        osqp_update_data_mat(solver,
                             m_P_sparse.valuePtr(), NULL, m_P_sparse.nonZeros(),
                             m_A_sparse.valuePtr(), NULL, m_A_sparse.nonZeros());

        // Update vectors in-place in OSQP workspace
        osqp_update_data_vec(solver, m_q.data(), m_l.data(), m_u.data());
    }

    // Solve QP
    OSQPInt exitflag = osqp_solve(solver);
    if (exitflag != 0) {
        return false;
    }

    // Return true if status is solved or solved inaccurate
    return (solver->info->status_val == OSQP_SOLVED || solver->info->status_val == OSQP_SOLVED_INACCURATE);
}

Eigen::VectorXd OsqpSolver::getSolution() const {
    Eigen::VectorXd sol(m_n);
    OSQPSolver* solver = static_cast<OSQPSolver*>(m_solver);
    if (solver && solver->solution && solver->solution->x) {
        for (int i = 0; i < m_n; ++i) {
            sol(i) = solver->solution->x[i];
        }
    } else {
        sol.setZero();
    }
    return sol;
}
