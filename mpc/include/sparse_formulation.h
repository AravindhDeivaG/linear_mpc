#ifndef SPARSE_FORMULATION_H
#define SPARSE_FORMULATION_H

#include "mpc_formulation.h"
#include "osqp_solver.h"
#include <iostream>
#include <memory>
#include <cassert>

class SparseFormulation : public MpcFormulation {
public:
    SparseFormulation(int n, int nx, int nu, double dt = 0.01);
    ~SparseFormulation() override;

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
    void getPredictedInputs(Eigen::VectorXd& U) override;

private:
    int n_;
    int nx_;
    int nu_;
    double dt_;
    
    std::unique_ptr<OsqpSolver> solver_;

    Eigen::MatrixXd A_, B_;
    Eigen::VectorXd x_min_, x_max_;
    Eigen::VectorXd u_min_, u_max_;
    Eigen::MatrixXd Q_, R_;

    Eigen::MatrixXd Sx_, Su_;
    Eigen::MatrixXd Q_full_, R_full_;
    Eigen::MatrixXd H_;
    Eigen::VectorXd g_;
    Eigen::MatrixXd M_;
    Eigen::VectorXd l_, u_;

    Eigen::VectorXd x_;
    Eigen::VectorXd x_ref_;
    Eigen::VectorXd u_opt_;

    // Pre-conditioning / Scaling transformations
    Eigen::VectorXd Tx_diag_, Tu_diag_;
    Eigen::VectorXd Tx_inv_diag_, Tu_inv_diag_;
    Eigen::MatrixXd A_scaled_, B_scaled_;
    Eigen::MatrixXd Q_scaled_, R_scaled_;
    Eigen::VectorXd x_min_scaled_, x_max_scaled_;
    Eigen::VectorXd u_min_scaled_, u_max_scaled_;

    bool is_setup_;
};

#endif // SPARSE_FORMULATION_H
