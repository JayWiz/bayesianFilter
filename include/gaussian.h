#ifndef GAUSSIAN_H
#define GAUSSIAN_H

#include <eigen3/Eigen/Dense>

class Gaussian {
    public:
        Gaussian(const Eigen::Matrix3f& p_cov, const Eigen::Vector3f& p_mu);
        Gaussian();

        Eigen::Matrix3f GetCovariance() const;
        Eigen::Vector3f GetExpectedValue() const;

        void SetExpectedValue(const Eigen::Vector3f& p_mu);
        void SetCovariance(const Eigen::Matrix3f& p_cov);

    private:
        Eigen::Matrix3f _cov;
        Eigen::Vector3f _mu;
} ;

#endif