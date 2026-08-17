#include "osqp_solver.h"
#include <osqp/osqp.h>
#include <iostream>
#include <stdexcept>

OsqpSolver::OsqpSolver(int n, int m)
    : m_n(n), m_m(m), m_solver(nullptr), m_is_initialized(false), m_matrices_need_update(false) {
    m_P.resize(m_n, m_n);
    m_P.setZero();
    m_q.resize(m_n);
    m_q.setZero();
    m_A.resize(m_m, m_n);
    m_A.setZero();
    m_l.resize(m_m);
    m_l.setZero();
    m_u.resize(m_m);
    m_u.setZero();
}

OsqpSolver::~OsqpSolver() {
    if (m_solver) {
        OSQPSolver* solver = static_cast<OSQPSolver*>(m_solver);
        osqp_cleanup(solver);
        m_solver = nullptr;
    }
}

void OsqpSolver::setHessian(const Eigen::MatrixXd& P) {
    m_P = P;
    m_matrices_need_update = true;
}

void OsqpSolver::setGradient(const Eigen::VectorXd& q) {
    m_q = q;
}

void OsqpSolver::setConstraintMatrix(const Eigen::MatrixXd& A) {
    m_A = A;
    m_matrices_need_update = true;
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
        if (m_P.rows() != m_n || m_P.cols() != m_n) {
            throw std::runtime_error("Hessian P dimension mismatch");
        }
        if (m_q.size() != m_n) {
            throw std::runtime_error("Gradient q dimension mismatch");
        }
        if (m_A.rows() != m_m || m_A.cols() != m_n) {
            throw std::runtime_error("Constraint matrix A dimension mismatch");
        }
        if (m_l.size() != m_m) {
            throw std::runtime_error("Lower bound l dimension mismatch");
        }
        if (m_u.size() != m_m) {
            throw std::runtime_error("Upper bound u dimension mismatch");
        }

        m_P_sparse.resize(m_n, m_n);
        m_P_sparse.reserve(Eigen::VectorXi::Constant(m_n, m_n));
        for (int col = 0; col < m_n; ++col) {
            for (int row = 0; row <= col; ++row) {
                m_P_sparse.insert(row, col) = m_P(row, col);
            }
        }
        m_P_sparse.makeCompressed();

        m_A_sparse.resize(m_m, m_n);
        m_A_sparse.reserve(Eigen::VectorXi::Constant(m_n, m_m));
        for (int col = 0; col < m_n; ++col) {
            for (int row = 0; row < m_m; ++row) {
                m_A_sparse.insert(row, col) = m_A(row, col);
            }
        }
        m_A_sparse.makeCompressed();

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

        OSQPSettings settings;
        osqp_set_default_settings(&settings);
        settings.verbose = 0;
        settings.eps_abs = 1e-5;
        settings.eps_rel = 1e-5;

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
        m_matrices_need_update = false;
    } else {
        if (m_matrices_need_update) {
            for (int k = 0; k < m_P_sparse.outerSize(); ++k) {
                for (decltype(m_P_sparse)::InnerIterator it(m_P_sparse, k); it; ++it) {
                    it.valueRef() = m_P(it.row(), it.col());
                }
            }

            for (int k = 0; k < m_A_sparse.outerSize(); ++k) {
                for (decltype(m_A_sparse)::InnerIterator it(m_A_sparse, k); it; ++it) {
                    it.valueRef() = m_A(it.row(), it.col());
                }
            }

            osqp_update_data_mat(solver,
                                 m_P_sparse.valuePtr(), NULL, m_P_sparse.nonZeros(),
                                 m_A_sparse.valuePtr(), NULL, m_A_sparse.nonZeros());
            m_matrices_need_update = false;
        }

        osqp_update_data_vec(solver, m_q.data(), m_l.data(), m_u.data());
    }

    OSQPInt exitflag = osqp_solve(solver);
    if (exitflag != 0) {
        return false;
    }

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

int OsqpSolver::getStatus() const {
    OSQPSolver* solver = static_cast<OSQPSolver*>(m_solver);
    if (solver && solver->info) return solver->info->status_val;
    return -1;
}

int OsqpSolver::getIterations() const {
    OSQPSolver* solver = static_cast<OSQPSolver*>(m_solver);
    if (solver && solver->info) return solver->info->iter;
    return 0;
}

double OsqpSolver::getPrimalResidual() const {
    OSQPSolver* solver = static_cast<OSQPSolver*>(m_solver);
    if (solver && solver->info) return solver->info->prim_res;
    return 0.0;
}

double OsqpSolver::getDualResidual() const {
    OSQPSolver* solver = static_cast<OSQPSolver*>(m_solver);
    if (solver && solver->info) return solver->info->dual_res;
    return 0.0;
}
