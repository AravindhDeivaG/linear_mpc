#include "sparse_formulation.h"
#include <iostream>
#include <cassert>
#include <cmath>
#include <chrono>

// Constructor: Pre-allocates memory for all Sparse MPC matrices given n, nx, nu
SparseFormulation::SparseFormulation(int n, int nx, int nu, double dt)
    : n_(n), nx_(nx), nu_(nu), dt_(dt), is_setup_(false) {
    
    // Allocate state, reference, and control vectors
    x_.resize(nx_);
    x_.setZero();
    x_ref_.resize(nx_);
    x_ref_.setZero();
    u_opt_.resize(nu_);
    u_opt_.setZero();

    // Allocate system dynamics matrices A and B
    A_.resize(nx_, nx_);
    A_.setZero();
    B_.resize(nx_, nu_);
    B_.setZero();

    // Allocate limits
    x_min_.resize(nx_);
    x_min_.setConstant(-1e4);
    x_max_.resize(nx_);
    x_max_.setConstant(1e4);

    u_min_.resize(nu_);
    u_min_.setConstant(-500.0);
    u_max_.resize(nu_);
    u_max_.setConstant(500.0);

    // Allocate single-step cost matrices
    Q_.resize(nx_, nx_);
    Q_ = Eigen::MatrixXd::Identity(nx_, nx_) * 10.0;
    R_.resize(nu_, nu_);
    R_ = Eigen::MatrixXd::Identity(nu_, nu_) * 0.1;

    int n_step = nu_ + nx_;
    int n_var = n_ * n_step;
    int n_dyn = n_ * nx_;
    int n_constr = n_var + n_dyn;

    // Pre-allocate selection matrices
    Sx_.resize(n_ * nx_, n_var);
    Sx_.setZero();
    Su_.resize(n_ * nu_, n_var);
    Su_.setZero();

    // Pre-allocate full cost matrices
    Q_full_.resize(n_ * nx_, n_ * nx_);
    Q_full_.setZero();
    R_full_.resize(n_ * nu_, n_ * nu_);
    R_full_.setZero();

    // Pre-allocate Hessian and gradient
    H_.resize(n_var, n_var);
    H_.setZero();
    g_.resize(n_var);
    g_.setZero();

    // Pre-allocate constraint matrix and bounds
    M_.resize(n_constr, n_var);
    M_.setZero();

    l_.resize(n_constr);
    l_.setZero();
    u_.resize(n_constr);
    u_.setZero();

    // Initialize OSQP Solver workspace
    solver_.reset(new OsqpSolver(n_var, n_constr));
}

SparseFormulation::~SparseFormulation() = default;

// Sets system matrices A (nx x nx) and B (nx x nu) with dimension assertions
void SparseFormulation::setSystemMatrices(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B) {
    assert(A.rows() == nx_ && A.cols() == nx_ && "Matrix A dimensions must match constructor nx");
    assert(B.rows() == nx_ && B.cols() == nu_ && "Matrix B dimensions must match constructor (nx, nu)");
    A_ = A;
    B_ = B;
}

// Sets state limits with dimension assertions
void SparseFormulation::setStateLimits(const Eigen::VectorXd& x_min, const Eigen::VectorXd& x_max) {
    assert(x_min.size() == nx_ && "x_min size must match constructor nx");
    assert(x_max.size() == nx_ && "x_max size must match constructor nx");
    x_min_ = x_min;
    x_max_ = x_max;
}

// Sets input limits with dimension assertions
void SparseFormulation::setInputLimits(const Eigen::VectorXd& u_min, const Eigen::VectorXd& u_max) {
    assert(u_min.size() == nu_ && "u_min size must match constructor nu");
    assert(u_max.size() == nu_ && "u_max size must match constructor nu");
    u_min_ = u_min;
    u_max_ = u_max;
}

// Sets cost matrices Q and R with dimension assertions
void SparseFormulation::setCostMatrices(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
    assert(Q.rows() == nx_ && Q.cols() == nx_ && "Matrix Q dimensions must match constructor nx");
    assert(R.rows() == nu_ && R.cols() == nu_ && "Matrix R dimensions must match constructor nu");
    Q_ = Q;
    R_ = R;
}

// Setup populates all pre-allocated Sparse MPC matrices with scaling transformations
void SparseFormulation::setup() {
    int n_step = nu_ + nx_;
    int n_var = n_ * n_step;
    int n_dyn = n_ * nx_;
    int n_constr = n_var + n_dyn;

    // Compute diagonal scaling transformation vectors
    Tx_diag_.resize(nx_);
    Tx_inv_diag_.resize(nx_);
    for (int i = 0; i < nx_; ++i) {
        double max_val = std::abs(x_max_(i));
        if (max_val < 1e-3 || max_val > 1e6) max_val = 1.0;
        Tx_diag_(i) = 1.0 / max_val;
        Tx_inv_diag_(i) = max_val;
    }

    Tu_diag_.resize(nu_);
    Tu_inv_diag_.resize(nu_);
    for (int j = 0; j < nu_; ++j) {
        double max_val = std::abs(u_max_(j));
        if (max_val < 1e-3 || max_val > 1e6) max_val = 1.0;
        Tu_diag_(j) = 1.0 / max_val;
        Tu_inv_diag_(j) = max_val;
    }

    Eigen::DiagonalMatrix<double, Eigen::Dynamic> Tx(Tx_diag_);
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> Tx_inv(Tx_inv_diag_);
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> Tu(Tu_diag_);
    Eigen::DiagonalMatrix<double, Eigen::Dynamic> Tu_inv(Tu_inv_diag_);

    // Compute scaled system dynamics and cost matrices
    A_scaled_ = Tx * A_ * Tx_inv;
    B_scaled_ = Tx * B_ * Tu_inv;
    Q_scaled_ = Tx_inv * Q_ * Tx_inv;
    R_scaled_ = Tu_inv * R_ * Tu_inv;

    // Normalize cost scaling factor so Hessian entries remain O(1) to O(10)
    double max_q = Q_scaled_.cwiseAbs().maxCoeff();
    if (max_q > 10.0) {
        Q_scaled_ /= (max_q / 10.0);
        R_scaled_ /= (max_q / 10.0);
    }

    x_min_scaled_ = Tx * x_min_;
    x_max_scaled_ = Tx * x_max_;
    u_min_scaled_ = Tu * u_min_;
    u_max_scaled_ = Tu * u_max_;

    // 1. Populate selection matrices Sx and Su
    Sx_.setZero();
    Su_.setZero();
    for (int k = 0; k < n_; ++k) {
        Sx_.block(k * nx_, k * n_step + nu_, nx_, nx_) = Eigen::MatrixXd::Identity(nx_, nx_);
        Su_.block(k * nu_, k * n_step, nu_, nu_) = Eigen::MatrixXd::Identity(nu_, nu_);
    }

    // 2. Populate full Q_full and R_full matrices using scaled cost
    Q_full_.setZero();
    R_full_.setZero();
    for (int k = 0; k < n_; ++k) {
        Q_full_.block(k * nx_, k * nx_, nx_, nx_) = Q_scaled_;
        R_full_.block(k * nu_, k * nu_, nu_, nu_) = R_scaled_;
    }

    // 3. Populate Hessian H = 2 * (Sx' * Q_full * Sx + Su' * R_full * Su)
    H_ = 2.0 * (Sx_.transpose() * Q_full_ * Sx_ + Su_.transpose() * R_full_ * Su_);

    // 4. Populate Constraint Matrix M using scaled A and B
    M_.setZero();
    // Top block: Box constraint identity
    M_.block(0, 0, n_var, n_var) = Eigen::MatrixXd::Identity(n_var, n_var);

    // Bottom block: Dynamics B_scaled*u_k + A_scaled*x_k - x_{k+1} = 0
    int dyn_offset = n_var;
    M_.block(dyn_offset, 0, nx_, nu_) = B_scaled_;
    M_.block(dyn_offset, nu_, nx_, nx_) = -Eigen::MatrixXd::Identity(nx_, nx_);

    for (int k = 1; k < n_; ++k) {
        int row_idx = dyn_offset + k * nx_;
        int prev_x_col = (k - 1) * n_step + nu_;
        int curr_u_col = k * n_step;
        int curr_x_col = k * n_step + nu_;

        M_.block(row_idx, prev_x_col, nx_, nx_) = A_scaled_;
        M_.block(row_idx, curr_u_col, nx_, nu_) = B_scaled_;
        M_.block(row_idx, curr_x_col, nx_, nx_) = -Eigen::MatrixXd::Identity(nx_, nx_);
    }

    // 5. Populate Lower and Upper bound vectors l and u
    l_.setZero();
    u_.setZero();

    for (int k = 0; k < n_; ++k) {
        l_.segment(k * n_step, nu_) = u_min_scaled_;
        u_.segment(k * n_step, nu_) = u_max_scaled_;

        l_.segment(k * n_step + nu_, nx_) = x_min_scaled_;
        u_.segment(k * n_step + nu_, nx_) = x_max_scaled_;
    }

    l_.segment(dyn_offset + nx_, (n_ - 1) * nx_).setZero();
    u_.segment(dyn_offset + nx_, (n_ - 1) * nx_).setZero();

    solver_->setHessian(H_);
    solver_->setConstraintMatrix(M_);

    is_setup_ = true;
    std::cout << "SparseFormulation setup completed (nx=" << nx_ << ", nu=" << nu_ << ", n=" << n_ << ")." << std::endl;
}

// Sets the current state x0 with dimension assertion
void SparseFormulation::setCurrentState(const Eigen::VectorXd& x) {
    assert(x.size() == nx_ && "Current state x vector size must match constructor nx");
    x_ = x;
}

// Sets the target reference state x_ref with dimension assertion
void SparseFormulation::setReferenceState(const Eigen::VectorXd& x_ref) {
    assert(x_ref.size() == nx_ && "Reference state x_ref vector size must match constructor nx");
    x_ref_ = x_ref;
}

// Gets the optimal control input u0
void SparseFormulation::getOptimalControl(Eigen::VectorXd& u) {
    u = u_opt_;
}

// Solves the Sparse MPC problem
void SparseFormulation::doControl() {
    if (!is_setup_ || !solver_) {
        std::cerr << "SparseFormulation doControl() failed: setup() not called!" << std::endl;
        return;
    }

    Eigen::VectorXd x_scaled = Tx_diag_.cwiseProduct(x_);
    Eigen::VectorXd x_ref_scaled = Tx_diag_.cwiseProduct(x_ref_);

    Eigen::VectorXd x_ref_full(n_ * nx_);
    for (int k = 0; k < n_; ++k) {
        x_ref_full.segment(k * nx_, nx_) = x_ref_scaled;
    }

    // Gradient g = -2 * Sx' * Q_full * x_ref_full
    g_ = -2.0 * Sx_.transpose() * Q_full_ * x_ref_full;

    // Update dynamics equality bound for k=0: B_scaled*u0 - x1 = -A_scaled*x0
    int dyn_offset = n_ * (nu_ + nx_);
    Eigen::VectorXd step0_rhs = -A_scaled_ * x_scaled;
    l_.segment(dyn_offset, nx_) = step0_rhs;
    u_.segment(dyn_offset, nx_) = step0_rhs;

    int n_step = nu_ + nx_;
    Eigen::VectorXd prev_sol = solver_->getSolution();

    if (prev_sol.size() == n_ * n_step && prev_sol.norm() > 1e-6) {
        Eigen::VectorXd z_shift(n_ * n_step);
        // Shift step k=0..N-2 forward by 1 time step
        for (int k = 0; k < n_ - 1; ++k) {
            z_shift.segment(k * n_step, n_step) = prev_sol.segment((k + 1) * n_step, n_step);
        }
        // Last step N-1: replicate input u_{N-2} and integrate state x_N
        int last_offset = (n_ - 1) * n_step;
        Eigen::VectorXd u_last = prev_sol.segment((n_ - 2) * n_step, nu_);
        Eigen::VectorXd x_last_prev = prev_sol.segment((n_ - 1) * n_step + nu_, nx_);
        Eigen::VectorXd x_next = A_scaled_ * x_last_prev + B_scaled_ * u_last;

        z_shift.segment(last_offset, nu_) = u_last;
        z_shift.segment(last_offset + nu_, nx_) = x_next;

        solver_->setWarmStart(z_shift);

        // Perform Dual Warm Start using actual previous dual multipliers y
        Eigen::VectorXd prev_dual = solver_->getDualSolution();
        if (prev_dual.size() == l_.size()) {
            solver_->setWarmStartDual(prev_dual);
        }
    }

    solver_->setGradient(g_);
    solver_->setLowerBound(l_);
    solver_->setUpperBound(u_);

    solver_->solve();

    int status = solver_->getStatus();
    if (status != 1 && status != 2) { // 1 = OSQP_SOLVED, 2 = OSQP_SOLVED_INACCURATE
        std::cerr << "[SparseFormulation Warning] OSQP did not converge! Status = " << status 
                  << " | Iterations = " << solver_->getIterations() 
                  << " | PrimRes = " << solver_->getPrimalResidual() 
                  << " | DualRes = " << solver_->getDualResidual() << std::endl;
    }

    Eigen::VectorXd sol = solver_->getSolution();
    if (sol.size() >= nu_) {
        // Unscale control output: u_opt = Tu_inv * u_scaled
        u_opt_ = Tu_inv_diag_.cwiseProduct(sol.head(nu_));
    }
}

// Retrieves predicted state trajectory X over horizon
void SparseFormulation::getPredictedStates(Eigen::VectorXd& X) {
    if (!is_setup_ || !solver_) {
        X.resize(nx_ * n_);
        X.setZero();
        return;
    }

    Eigen::VectorXd sol = solver_->getSolution();
    int n_step = nu_ + nx_;
    X.resize(n_ * nx_);

    for (int k = 0; k < n_; ++k) {
        Eigen::VectorXd x_k_scaled = sol.segment(k * n_step + nu_, nx_);
        X.segment(k * nx_, nx_) = Tx_inv_diag_.cwiseProduct(x_k_scaled);
    }
}

// Retrieves predicted control input trajectory U over horizon
void SparseFormulation::getPredictedInputs(Eigen::VectorXd& U) {
    if (!is_setup_ || !solver_) {
        U.resize(nu_ * n_);
        U.setZero();
        return;
    }

    Eigen::VectorXd sol = solver_->getSolution();
    int n_step = nu_ + nx_;
    U.resize(n_ * nu_);

    for (int k = 0; k < n_; ++k) {
        Eigen::VectorXd u_k_scaled = sol.segment(k * n_step, nu_);
        U.segment(k * nu_, nu_) = Tu_inv_diag_.cwiseProduct(u_k_scaled);
    }
}

int SparseFormulation::getIterations() const {
    return solver_ ? solver_->getIterations() : 0;
}

int SparseFormulation::getStatus() const {
    return solver_ ? solver_->getStatus() : -1;
}

const char* SparseFormulation::getStatusString() const {
    return solver_ ? solver_->getStatusString() : "UNINITIALIZED";
}

double SparseFormulation::getObjectiveValue() const {
    return solver_ ? solver_->getObjectiveValue() : 0.0;
}

double SparseFormulation::getPrimalResidual() const {
    return solver_ ? solver_->getPrimalResidual() : 0.0;
}

double SparseFormulation::getDualResidual() const {
    return solver_ ? solver_->getDualResidual() : 0.0;
}

Eigen::VectorXd SparseFormulation::getRawSolution() const {
    return solver_ ? solver_->getSolution() : Eigen::VectorXd::Zero(n_ * (nu_ + nx_));
}
