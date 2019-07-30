#include "motionmodel.h"
#include <cmath>
#include "gaussian.h"
#define PI 3.14159265
#include <iostream>

MotionModel::MotionModel() {

}

MotionModel::~MotionModel() {

}

Gaussian MotionModel::CalculateMotion(Eigen::Vector3f p_odometry, Gaussian p_gaussian, Eigen::Matrix3f p_motionNoise) {
    Eigen::Matrix3f sigma_old = p_gaussian.GetCovariance();
    Eigen::Vector3f mu_old = p_gaussian.GetExpectedValue();

    float_t delta_rot_1 = p_odometry(0);
    float_t delta_rot_2 = p_odometry(1);
    float_t delta_trans = p_odometry(2);

    float_t x = mu_old(0);
    float_t y = mu_old(1);
    float_t theta = mu_old(2);

    Eigen::Vector3f mu_new = Eigen::Vector3f();
    mu_new <<    x + delta_trans * std::cos((theta + delta_rot_1)), 
                    y + delta_trans * std::sin((theta + delta_rot_1)), 
                    theta + delta_rot_1 + delta_rot_2;

    Eigen::Matrix3f G = Eigen::Matrix3f();
    G <<    1, 0, -delta_trans * std::sin((theta + delta_rot_1)),
            0, 1, +delta_trans * std::cos((theta + delta_rot_1)),
            0, 0, 1;

    // std::cout << G << std::endl;

    Eigen::Matrix3f sigma_new = G * sigma_old * G.transpose() + p_motionNoise;

    Gaussian resultGaussian = Gaussian(sigma_new, mu_new);

    return resultGaussian;
}