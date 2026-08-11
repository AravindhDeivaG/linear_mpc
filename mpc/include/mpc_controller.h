#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include "osqp_solver.h"
#include "Eigen/Dense"
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>

class MpcController {
public:
    /*
    @brief Constructor
    @param n Horizon (Number of steps the optimizer will solve for)
    @param dt Time step
    */
    MpcController(int n=10, double dt=0.01);

    // Destructor
    ~MpcController();

    // Function to perform control action
    void doControl();

    // Function to set current state
    void setCurrentState(Eigen::VectorXd x);

    // Function to set reference state
    void setReferenceState(Eigen::VectorXd x_ref);

    // Get optimal control action
    void getOptimalControl(Eigen::VectorXd& u);

private:
    // Time step
    double dt_;
    
    // Horizon
    int n_;
    
    // OSQP solver instance
    OsqpSolver solver_;

    // State space matrices
    Eigen::MatrixXd A_, B_;

    // Combined state space matrices
    Eigen::MatrixXd Sx_, Su_;

    // Current state
    Eigen::VectorXd x_;

    // Reference state
    Eigen::VectorXd x_ref_;

    // Current optimal control command
    Eigen::VectorXd u_opt_;

    /*
     MPC optimization matrices
    */
    // Hessian matrix
    Eigen::MatrixXd H_;
    // Gradient vector
    Eigen::VectorXd g_;
    // Constraint matrix
    Eigen::MatrixXd M_;
    // Lower bound
    Eigen::VectorXd l_;
    // Upper bound
    Eigen::VectorXd u_;

    // State limits
    Eigen::Vector4d x_min_, x_max_;

    // Input limits
    Eigen::Vector2d u_min_, u_max_;

    /*
     Cost function related variables
    */
    Eigen::MatrixXd Q_, R_;

    // Projector matrix to create copy of a variable multiple times
    Eigen::MatrixXd state_projector_;
    Eigen::MatrixXd input_projector_;

    
};

#endif // MPC_CONTROLLER_H
