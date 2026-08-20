#include "osqp_solver.h"
#include <osqp/osqp.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <stdexcept>
#include <chrono>

bool OsqpConfig::loadFromYaml(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        size_t comment_pos = line.find('#');
        if (comment_pos != std::string::npos) line = line.substr(0, comment_pos);

        size_t colon_pos = line.find(':');
        if (colon_pos == std::string::npos) continue;

        std::string key = line.substr(0, colon_pos);
        std::string val = line.substr(colon_pos + 1);

        auto trim = [](std::string& s) {
            s.erase(0, s.find_first_not_of(" \t\r\n"));
            s.erase(s.find_last_not_of(" \t\r\n") + 1);
        };
        trim(key);
        trim(val);

        if (key.empty() || val.empty()) continue;

        auto parse_bool = [](const std::string& v) {
            return (v == "true" || v == "1" || v == "True" || v == "YES" || v == "yes");
        };

        if (key == "verbose") verbose = parse_bool(val);
        else if (key == "enable_timing") enable_timing = parse_bool(val);
        else if (key == "time_threshold_ms") time_threshold_ms = std::stod(val);
        else if (key == "check_termination") check_termination = std::stoi(val);
        else if (key == "rho") rho = std::stod(val);
        else if (key == "adaptive_rho") adaptive_rho = parse_bool(val);
        else if (key == "eps_abs") eps_abs = std::stod(val);
        else if (key == "eps_rel") eps_rel = std::stod(val);
        else if (key == "max_iter") max_iter = std::stoi(val);
        else if (key == "polishing") polishing = parse_bool(val);
    }
    return true;
}

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

    // Try loading default osqp_impl/config/osqp_params.yaml if present
    if (!m_config.loadFromYaml("osqp_impl/config/osqp_params.yaml")) {
        if (!m_config.loadFromYaml("../osqp_impl/config/osqp_params.yaml")) {
            m_config.loadFromYaml("config/osqp_params.yaml");
        }
    }
}

void OsqpSolver::setConfig(const OsqpConfig& config) {
    m_config = config;
}

bool OsqpSolver::loadConfig(const std::string& yaml_path) {
    return m_config.loadFromYaml(yaml_path);
}

OsqpConfig OsqpSolver::getConfig() const {
    return m_config;
}

void OsqpSolver::setVerbose(bool verbose) {
    m_config.verbose = verbose;
}

void OsqpSolver::setLogTiming(bool enable, double threshold_ms) {
    m_config.enable_timing = enable;
    m_config.time_threshold_ms = threshold_ms;
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

void OsqpSolver::setWarmStart(const Eigen::VectorXd& x) {
    OSQPSolver* solver = static_cast<OSQPSolver*>(m_solver);
    if (solver && m_is_initialized && x.size() == m_n) {
        osqp_warm_start(solver, x.data(), NULL);
    }
}

void OsqpSolver::setWarmStartDual(const Eigen::VectorXd& y) {
    OSQPSolver* solver = static_cast<OSQPSolver*>(m_solver);
    if (solver && m_is_initialized && y.size() == m_m) {
        osqp_warm_start(solver, NULL, y.data());
    }
}

namespace {
class OsqpScopedTimer {
public:
    OsqpScopedTimer(const OSQPSolver* solver, bool enable = false, double threshold_ms = 1.0)
        : m_solver(solver), m_enable(enable), m_threshold_ms(threshold_ms), 
          m_start(std::chrono::high_resolution_clock::now()) {}

    ~OsqpScopedTimer() {
        if (!m_enable) return;
        auto end = std::chrono::high_resolution_clock::now();
        double solve_ms = std::chrono::duration<double, std::milli>(end - m_start).count();
        if (solve_ms > m_threshold_ms) {
            int iters = (m_solver && m_solver->info) ? m_solver->info->iter : 0;
            std::cout << "[OsqpEngine Performance Warning] osqp_solve engine took " << solve_ms 
                      << " ms (> " << m_threshold_ms << " ms limit!) | Iterations: " << iters << std::endl;
        }
    }

private:
    const OSQPSolver* m_solver;
    bool m_enable;
    double m_threshold_ms;
    std::chrono::high_resolution_clock::time_point m_start;
};
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
        for (int col = 0; col < m_n; ++col) {
            for (int row = 0; row <= col; ++row) {
                double val = m_P(row, col);
                if (std::abs(val) > 1e-12) {
                    m_P_sparse.insert(row, col) = val;
                }
            }
        }
        m_P_sparse.makeCompressed();

        m_A_sparse.resize(m_m, m_n);
        for (int col = 0; col < m_n; ++col) {
            for (int row = 0; row < m_m; ++row) {
                double val = m_A(row, col);
                if (std::abs(val) > 1e-12) {
                    m_A_sparse.insert(row, col) = val;
                }
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
        settings.verbose = m_config.verbose ? 1 : 0;
        settings.check_termination = m_config.check_termination;
        settings.rho = m_config.rho;
        settings.adaptive_rho = m_config.adaptive_rho ? 1 : 0;
        settings.eps_abs = m_config.eps_abs;
        settings.eps_rel = m_config.eps_rel;
        settings.max_iter = m_config.max_iter;
        settings.polishing = m_config.polishing ? 1 : 0;

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

    if (m_config.verbose) {
        std::cout << "\n==========================================================================================\n"
                  << "[OsqpSolver] --- STARTING OSQP SOLVE CYCLE ---\n"
                  << "==========================================================================================\n" << std::endl;
    }

    OSQPInt exitflag;
    {
        OsqpScopedTimer timer(solver, m_config.enable_timing, m_config.time_threshold_ms);
        exitflag = osqp_solve(solver);
    }
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

Eigen::VectorXd OsqpSolver::getDualSolution() const {
    Eigen::VectorXd sol(m_m);
    OSQPSolver* solver = static_cast<OSQPSolver*>(m_solver);
    if (solver && solver->solution && solver->solution->y) {
        for (int i = 0; i < m_m; ++i) {
            sol(i) = solver->solution->y[i];
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

const char* OsqpSolver::getStatusString() const {
    OSQPSolver* solver = static_cast<OSQPSolver*>(m_solver);
    if (solver && solver->info) return solver->info->status;
    return "UNKNOWN";
}

int OsqpSolver::getIterations() const {
    OSQPSolver* solver = static_cast<OSQPSolver*>(m_solver);
    if (solver && solver->info) return solver->info->iter;
    return 0;
}

double OsqpSolver::getObjectiveValue() const {
    OSQPSolver* solver = static_cast<OSQPSolver*>(m_solver);
    if (solver && solver->info) return solver->info->obj_val;
    return 0.0;
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
