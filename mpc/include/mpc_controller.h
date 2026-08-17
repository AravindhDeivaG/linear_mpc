#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include "mpc_formulation.h"
#include "dense_formulation.h"
#include "sparse_formulation.h"
#include <memory>

enum class FormulationType {
    DENSE,
    SPARSE
};

class MpcController {
public:
    MpcController(int n = 10, int nx = 4, int nu = 2, FormulationType type = FormulationType::DENSE);
    ~MpcController();

    void setFormulation(FormulationType type);
    FormulationType getFormulationType() const;

    int getHorizon() const;
    int getNx() const;
    int getNu() const;

    void setSystemMatrices(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B);
    void setStateLimits(const Eigen::VectorXd& x_min, const Eigen::VectorXd& x_max);
    void setInputLimits(const Eigen::VectorXd& u_min, const Eigen::VectorXd& u_max);
    void setCostMatrices(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);

    void setup();

    void setCurrentState(const Eigen::VectorXd& x);
    void setReferenceState(const Eigen::VectorXd& x_ref);
    void doControl();
    void getOptimalControl(Eigen::VectorXd& u);
    void getPredictedStates(Eigen::VectorXd& X);
    void getPredictedInputs(Eigen::VectorXd& U);

private:
    int n_;
    int nx_;
    int nu_;
    FormulationType type_;
    std::unique_ptr<MpcFormulation> formulation_;
};

#endif // MPC_CONTROLLER_H
