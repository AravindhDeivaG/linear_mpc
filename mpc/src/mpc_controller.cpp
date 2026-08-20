#include "mpc_controller.h"
#include <iostream>

MpcController::MpcController(int n, int nx, int nu, FormulationType type)
    : n_(n), nx_(nx), nu_(nu), type_(type) {
    setFormulation(type_);
}

MpcController::~MpcController() = default;

void MpcController::setFormulation(FormulationType type) {
    type_ = type;
    if (type_ == FormulationType::DENSE) {
        formulation_.reset(new DenseFormulation(n_, nx_, nu_));
    } else {
        formulation_.reset(new SparseFormulation(n_, nx_, nu_));
    }
}

FormulationType MpcController::getFormulationType() const {
    return type_;
}

int MpcController::getHorizon() const {
    return n_;
}

int MpcController::getNx() const {
    return nx_;
}

int MpcController::getNu() const {
    return nu_;
}

void MpcController::setSystemMatrices(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B) {
    if (formulation_) formulation_->setSystemMatrices(A, B);
}

void MpcController::setStateLimits(const Eigen::VectorXd& x_min, const Eigen::VectorXd& x_max) {
    if (formulation_) formulation_->setStateLimits(x_min, x_max);
}

void MpcController::setInputLimits(const Eigen::VectorXd& u_min, const Eigen::VectorXd& u_max) {
    if (formulation_) formulation_->setInputLimits(u_min, u_max);
}

void MpcController::setCostMatrices(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R) {
    if (formulation_) formulation_->setCostMatrices(Q, R);
}

void MpcController::setup() {
    if (formulation_) formulation_->setup();
}

void MpcController::setCurrentState(const Eigen::VectorXd& x) {
    if (formulation_) formulation_->setCurrentState(x);
}

void MpcController::setReferenceState(const Eigen::VectorXd& x_ref) {
    if (formulation_) formulation_->setReferenceState(x_ref);
}

void MpcController::doControl() {
    if (formulation_) formulation_->doControl();
}

void MpcController::getOptimalControl(Eigen::VectorXd& u) {
    if (formulation_) formulation_->getOptimalControl(u);
}

void MpcController::getPredictedStates(Eigen::VectorXd& X) {
    if (formulation_) formulation_->getPredictedStates(X);
}

void MpcController::getPredictedInputs(Eigen::VectorXd& U) {
    if (formulation_) formulation_->getPredictedInputs(U);
}

int MpcController::getIterations() const {
    return formulation_ ? formulation_->getIterations() : 0;
}

int MpcController::getStatus() const {
    return formulation_ ? formulation_->getStatus() : -1;
}

const char* MpcController::getStatusString() const {
    return formulation_ ? formulation_->getStatusString() : "UNINITIALIZED";
}

double MpcController::getObjectiveValue() const {
    return formulation_ ? formulation_->getObjectiveValue() : 0.0;
}

double MpcController::getPrimalResidual() const {
    return formulation_ ? formulation_->getPrimalResidual() : 0.0;
}

double MpcController::getDualResidual() const {
    return formulation_ ? formulation_->getDualResidual() : 0.0;
}

Eigen::VectorXd MpcController::getRawSolution() const {
    return formulation_ ? formulation_->getRawSolution() : Eigen::VectorXd::Zero(0);
}
