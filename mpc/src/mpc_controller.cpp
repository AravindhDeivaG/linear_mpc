#include "mpc_controller.h"
#include <iostream>

// Constructor
MpcController::MpcController(int n, double dt) : solver_(2*n, 6*n) {
    
    // Init time step and horizon
    dt_ = dt;
    n_ = n;

    // Initialize state space matrix
    A_.resize(4,4);
    B_.resize(4,2);

    A_.setIdentity();
    B_.setZero();

    A_(0,2) = dt_;
    A_(1,3) = dt_;
    B_(0,0) = 0.5*dt_*dt_;
    B_(1,1) = 0.5*dt_*dt_;
    B_(2,0) = dt_;
    B_(3,1) = dt_;

    std::cout<<"A : \n"<<A_<<std::endl;
    std::cout<<"B : \n"<<B_<<std::endl;

    // Initialize state
    x_.resize(4);
    u_.resize(2);

    // Initialize combined state space matrix size
    Sx_.resize(4*n_, 4);
    Su_.resize(4*n_, 2*n_);

    Sx_.setZero();
    Su_.setZero();

    for(int i=0;i<n_;i++)
    {
        Sx_.block(4*i,0,4,4) = A_.pow(i+1);

        for(int j=0;j<i+1;j++)
        {
            Su_.block(4*i,2*j,4,2) = A_.pow(i-j)*B_;
        }
    }

    // Cost function matrices
    Q_.resize(4*n_,4*n_);
    R_.resize(2*n_,2*n_);

    Q_.setZero();
    R_.setZero();

    for(int i=0;i<n_;i++)
    {
        Q_.block(4*i,4*i,2,2) = Eigen::MatrixXd::Identity(2,2)*10;
        Q_.block(4*i+2,4*i+2,2,2) = Eigen::MatrixXd::Identity(2,2)*0.1;
        R_.block(2*i,2*i,2,2) = Eigen::MatrixXd::Identity(2,2)*0.000;
    }

    // Hessian matrix H = 2*(Su'*Q*Su + R)
    H_ = 2.0 * (Su_.transpose()*Q_*Su_ + R_);

    // Constraint matrix M
    M_.resize(6*n_,2*n_);
    M_.setZero();
    M_.block(0,0,2*n_,2*n_) = Eigen::MatrixXd::Identity(2*n_,2*n_);
    M_.block(2*n_,0,4*n_,2*n_) = Su_;

    // Lower and upper limit vectors for constraints
    l_.resize(6*n_,1);
    u_.resize(6*n_,1);

    l_.setZero();
    u_.setZero();
    
    // State limits for position and velocity
    x_min_.resize(4);
    x_max_.resize(4);

    x_min_ << -10000,-10000,-200,-200;
    x_max_ << 10000,10000,200,200;
    
    // Input limits for acceleration
    u_min_.resize(2);
    u_max_.resize(2);

    u_min_ << -500,-500;
    u_max_ << 500,500;
    
    for(int i = 0;i<n_;i++)
    {
        l_.block(2*i,0,2,1) = u_min_;    
        u_.block(2*i,0,2,1) = u_max_;        
    }

    //  Initialize projector matrix for input and state
    state_projector_.resize(4*n_,4);
    input_projector_.resize(2*n_,2);

    state_projector_.setZero();
    input_projector_.setZero();

    for(int i=0;i<n_;i++)
    {
        state_projector_.block(4*i,0,4,4) = Eigen::MatrixXd::Identity(4,4);
        input_projector_.block(2*i,0,2,2) = Eigen::MatrixXd::Identity(2,2);
    }

    std::cout << "MpcController initialized." << std::endl;
}

// Destructor
MpcController::~MpcController() {
    std::cout << "MpcController destroyed." << std::endl;
}

// Function to set current state
void MpcController::setCurrentState(Eigen::VectorXd x) {
    x_ = x;
}

// Function to set reference state
void MpcController::setReferenceState(Eigen::VectorXd x_ref) {
    x_ref_ = x_ref;
}

// Function to get optimal control action
void MpcController::getOptimalControl(Eigen::VectorXd& u) {
    u = u_opt_;
}

// Function to perform control action
void MpcController::doControl() {
    
    /*
    Gradient g = 2 * Su' * Q * (Sx * x - x_ref)
    x_ref is multiplied by projector to create copies of itself
    */
    g_ = 2 * Su_.transpose() * Q_ * (Sx_ * x_ - state_projector_*x_ref_);
    
    // Lower and upper bound update for state constraints (size 4*n_)
    l_.block(2*n_, 0, 4*n_, 1) = -Sx_*x_ + state_projector_*x_min_;
    
    u_.block(2*n_, 0, 4*n_, 1) = -Sx_*x_ + state_projector_*x_max_;
    
    // Set H and g for solver
    solver_.setHessian(H_);
    solver_.setGradient(g_);
    solver_.setConstraintMatrix(M_);
    solver_.setLowerBound(l_);
    solver_.setUpperBound(u_);
    
    // solve 
    solver_.solve();
    
    // get solution
    u_opt_ = solver_.getSolution();
    
    
    // std::cout << "Control applied: " << u_[0] << " " << u_[1] << std::endl;
}

void MpcController::getPredictedStates(Eigen::VectorXd& X) {
    X = Sx_ * x_ + Su_ * u_opt_;
}

int MpcController::getHorizon() const {
    return n_;
}

