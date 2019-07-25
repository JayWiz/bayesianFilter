#include "gaussian.h"

Gaussian::Gaussian(const Eigen::Matrix3f& p_cov, const Eigen::Vector3f& p_mu) {
    _cov = p_cov;
    _mu = p_mu;
}

Gaussian::Gaussian() {
    
}

Eigen::Vector3f Gaussian::GetExpectedValue() const {
    return _mu;
}

Eigen::Matrix3f Gaussian::GetCovariance() const {
    return _cov;
}

void Gaussian::SetExpectedValue(const Eigen::Vector3f& p_mu) {
    _mu = p_mu;
}

void Gaussian::SetCovariance(const Eigen::Matrix3f& p_cov) {
    _cov = p_cov;
}

