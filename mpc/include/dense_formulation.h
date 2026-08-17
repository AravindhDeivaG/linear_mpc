#ifndef DENSE_FORMULATION_H
#define DENSE_FORMULATION_H

#include "mpc_formulation.h"
#include "osqp_solver.h"
#include <iostream>
#include <memory>
#include <cassert>
#include <unsupported/Eigen/MatrixFunctions>

class DenseFormulation : public MpcFormulation {
public:
    DenseFormulation(int n, int nx, int nu);
    ~DenseFormulation() override;

    void setSystemMatrices(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B) override;
    void setStateLimits(const Eigen::VectorXd& x_min, const Eigen::VectorXd& x_max) override;
    void setInputLimits(const Eigen::VectorXd& u_min, const Eigen::VectorXd& u_max) override;
    void setCostMatrices(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) override;

    void setup() override;

    void setCurrentState(const Eigen::VectorXd& x) override;
    void setReferenceState(const Eigen::VectorXd& x_ref) override;
    void doControl() override;
    void getOptimalControl(Eigen::VectorXd& u) override;
    void getPredictedStates(Eigen::VectorXd& X) override;

private:
    int n_;
    int nx_;
    int nu_;
    
    std::unique_ptr<OsqpSolver> solver_;

    Eigen::MatrixXd A_, B_;
    Eigen::MatrixXd Sx_, Su_;
    Eigen::VectorXd x_;
    Eigen::VectorXd x_ref_;
    Eigen::VectorXd u_opt_;

    Eigen::MatrixXd H_;
    Eigen::VectorXd g_;
    Eigen::MatrixXd M_;
    Eigen::VectorXd l_;
    Eigen::VectorXd u_;

    Eigen::VectorXd x_min_, x_max_;
    Eigen::VectorXd u_min_, u_max_;

    Eigen::MatrixXd Q_, R_;
    Eigen::MatrixXd Q_full_, R_full_;
    Eigen::MatrixXd state_projector_;
    Eigen::MatrixXd input_projector_;

    bool is_setup_;
};

#endif // DENSE_FORMULATION_H
