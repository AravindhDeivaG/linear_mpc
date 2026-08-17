#include "dense_formulation.h"
#include <iostream>
#include <cassert>

// Constructor: Pre-allocates memory for all dense MPC matrices given n, nx, nu
DenseFormulation::DenseFormulation(int n, int nx, int nu)
    : n_(n), nx_(nx), nu_(nu), is_setup_(false) {
    
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

    // Pre-allocate condensed state-space matrices
    Sx_.resize(nx_ * n_, nx_);
    Sx_.setZero();
    Su_.resize(nx_ * n_, nu_ * n_);
    Su_.setZero();

    // Pre-allocate full cost matrices
    Q_full_.resize(nx_ * n_, nx_ * n_);
    Q_full_.setZero();
    R_full_.resize(nu_ * n_, nu_ * n_);
    R_full_.setZero();

    // Pre-allocate Hessian and gradient
    H_.resize(nu_ * n_, nu_ * n_);
    H_.setZero();
    g_.resize(nu_ * n_);
    g_.setZero();

    // Pre-allocate constraint matrix and bounds
    int n_var = nu_ * n_;
    int n_constr = (nu_ + nx_) * n_;
    M_.resize(n_constr, n_var);
    M_.setZero();

    l_.resize(n_constr);
    l_.setZero();
    u_.resize(n_constr);
    u_.setZero();

    // Pre-allocate state and input projectors
    state_projector_.resize(nx_ * n_, nx_);
    state_projector_.setZero();
    input_projector_.resize(nu_ * n_, nu_);
    input_projector_.setZero();

    // Initialize OSQP Solver workspace
    solver_.reset(new OsqpSolver(n_var, n_constr));
}

DenseFormulation::~DenseFormulation() = default;

// Sets system matrices A (nx x nx) and B (nx x nu) with dimension assertions
void DenseFormulation::setSystemMatrices(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B) {
    assert(A.rows() == nx_ && A.cols() == nx_ && "Matrix A dimensions must match constructor nx");
    assert(B.rows() == nx_ && B.cols() == nu_ && "Matrix B dimensions must match constructor (nx, nu)");
    A_ = A;
    B_ = B;
}

// Sets state lower and upper limits with dimension assertions
void DenseFormulation::setStateLimits(const Eigen::VectorXd& x_min, const Eigen::VectorXd& x_max) {
    assert(x_min.size() == nx_ && "x_min size must match constructor nx");
    assert(x_max.size() == nx_ && "x_max size must match constructor nx");
    x_min_ = x_min;
    x_max_ = x_max;
}

// Sets input lower and upper limits with dimension assertions
void DenseFormulation::setInputLimits(const Eigen::VectorXd& u_min, const Eigen::VectorXd& u_max) {
    assert(u_min.size() == nu_ && "u_min size must match constructor nu");
    assert(u_max.size() == nu_ && "u_max size must match constructor nu");
    u_min_ = u_min;
    u_max_ = u_max;
}

// Sets cost matrices Q and R with dimension assertions
void DenseFormulation::setCostMatrices(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
    assert(Q.rows() == nx_ && Q.cols() == nx_ && "Matrix Q dimensions must match constructor nx");
    assert(R.rows() == nu_ && R.cols() == nu_ && "Matrix R dimensions must match constructor nu");
    Q_ = Q;
    R_ = R;
}

// Setup populates all pre-allocated MPC matrices without reallocating memory
void DenseFormulation::setup() {
    // 1. Populate Sx and Su
    Sx_.setZero();
    Su_.setZero();
    for (int i = 0; i < n_; ++i) {
        Sx_.block(i * nx_, 0, nx_, nx_) = A_.pow(i + 1);
        for (int j = 0; j <= i; ++j) {
            Su_.block(i * nx_, j * nu_, nx_, nu_) = A_.pow(i - j) * B_;
        }
    }

    // 2. Populate full diagonal Q_full and R_full matrices
    Q_full_.setZero();
    R_full_.setZero();
    for (int i = 0; i < n_; ++i) {
        Q_full_.block(i * nx_, i * nx_, nx_, nx_) = Q_;
        R_full_.block(i * nu_, i * nu_, nu_, nu_) = R_;
    }

    // 3. Populate Hessian H = 2 * (Su' * Q_full * Su + R_full)
    H_ = 2.0 * (Su_.transpose() * Q_full_ * Su_ + R_full_);

    // 4. Populate Constraint matrix M (input limits top, state limits bottom)
    int n_var = nu_ * n_;
    M_.setZero();
    M_.block(0, 0, n_var, n_var) = Eigen::MatrixXd::Identity(n_var, n_var);
    M_.block(n_var, 0, nx_ * n_, n_var) = Su_;

    // 5. Populate input bound limits in l and u
    l_.setZero();
    u_.setZero();
    for (int i = 0; i < n_; ++i) {
        l_.block(i * nu_, 0, nu_, 1) = u_min_;
        u_.block(i * nu_, 0, nu_, 1) = u_max_;
    }

    // 6. Populate Projector matrices
    state_projector_.setZero();
    input_projector_.setZero();
    for (int i = 0; i < n_; ++i) {
        state_projector_.block(i * nx_, 0, nx_, nx_) = Eigen::MatrixXd::Identity(nx_, nx_);
        input_projector_.block(i * nu_, 0, nu_, nu_) = Eigen::MatrixXd::Identity(nu_, nu_);
    }

    is_setup_ = true;
    std::cout << "DenseFormulation setup completed (nx=" << nx_ << ", nu=" << nu_ << ", n=" << n_ << ")." << std::endl;
}

// Sets the current state x0 with dimension assertion
void DenseFormulation::setCurrentState(const Eigen::VectorXd& x) {
    assert(x.size() == nx_ && "Current state x vector size must match constructor nx");
    x_ = x;
}

// Sets the target reference state x_ref with dimension assertion
void DenseFormulation::setReferenceState(const Eigen::VectorXd& x_ref) {
    assert(x_ref.size() == nx_ && "Reference state x_ref vector size must match constructor nx");
    x_ref_ = x_ref;
}

// Gets the optimal control input u0
void DenseFormulation::getOptimalControl(Eigen::VectorXd& u) {
    u = u_opt_;
}

// Computes optimal control action via OSQP solver
void DenseFormulation::doControl() {
    if (!is_setup_ || !solver_) {
        std::cerr << "DenseFormulation doControl() failed: setup() not called!" << std::endl;
        return;
    }

    // Gradient g = 2 * Su' * Q_full * (Sx * x - state_projector * x_ref)
    g_ = 2.0 * Su_.transpose() * Q_full_ * (Sx_ * x_ - state_projector_ * x_ref_);

    // Update state bounds (bottom part of l_ and u_)
    int n_var = nu_ * n_;
    l_.block(n_var, 0, nx_ * n_, 1) = -Sx_ * x_ + state_projector_ * x_min_;
    u_.block(n_var, 0, nx_ * n_, 1) = -Sx_ * x_ + state_projector_ * x_max_;

    solver_->setHessian(H_);
    solver_->setGradient(g_);
    solver_->setConstraintMatrix(M_);
    solver_->setLowerBound(l_);
    solver_->setUpperBound(u_);

    solver_->solve();
    Eigen::VectorXd sol = solver_->getSolution();
    if (sol.size() >= nu_) {
        u_opt_ = sol.head(nu_);
    }
}

// Returns predicted state trajectory over horizon
void DenseFormulation::getPredictedStates(Eigen::VectorXd& X) {
    if (!is_setup_ || !solver_) {
        X.resize(nx_ * n_);
        X.setZero();
        return;
    }
    Eigen::VectorXd U_full = solver_->getSolution();
    X = Sx_ * x_ + Su_ * U_full;
}
