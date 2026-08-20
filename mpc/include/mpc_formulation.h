#ifndef MPC_FORMULATION_H
#define MPC_FORMULATION_H

#include "Eigen/Dense"

class MpcFormulation {
public:
    virtual ~MpcFormulation() = default;

    virtual void setSystemMatrices(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B) = 0;
    virtual void setStateLimits(const Eigen::VectorXd& x_min, const Eigen::VectorXd& x_max) = 0;
    virtual void setInputLimits(const Eigen::VectorXd& u_min, const Eigen::VectorXd& u_max) = 0;
    virtual void setCostMatrices(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) = 0;

    virtual void setup() = 0;

    virtual void setCurrentState(const Eigen::VectorXd& x) = 0;
    virtual void setReferenceState(const Eigen::VectorXd& x_ref) = 0;
    virtual void doControl() = 0;
    virtual void getOptimalControl(Eigen::VectorXd& u) = 0;
    virtual void getPredictedStates(Eigen::VectorXd& X) = 0;
    virtual void getPredictedInputs(Eigen::VectorXd& U) = 0;
    virtual int getIterations() const = 0;
    virtual int getStatus() const = 0;
    virtual const char* getStatusString() const = 0;
    virtual double getObjectiveValue() const = 0;
    virtual double getPrimalResidual() const = 0;
    virtual double getDualResidual() const = 0;
    virtual Eigen::VectorXd getRawSolution() const = 0;
};

#endif // MPC_FORMULATION_H
