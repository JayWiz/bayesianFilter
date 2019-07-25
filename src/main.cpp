#include <iostream>
#include "motionmodel.h"
#include <eigen3/Eigen/Dense>

int main(void) {
    Eigen::Vector3f state1(3);
    Eigen::Vector3f action1(3);
    Eigen::Matrix3f cov1;
    Eigen::Matrix3f Q;
    cov1 << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    Q << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;
    
    
    action1 << 12, 7, 6;
    state1 << 2, 4, 3;

    Gaussian gaussian1 = Gaussian(cov1, state1);

    MotionModel m = MotionModel();
    Gaussian gaussian2 = m.CalculateMotion(action1, gaussian1, Q);
    std::cout << gaussian2.GetExpectedValue() << std::endl;
    std::cout << gaussian2.GetCovariance() << std::endl;
    Gaussian gaussian3 = m.CalculateMotion(action1, gaussian2, Q);
    std::cout << gaussian3.GetCovariance();
    //KalmanFilter k = KalmanFilter(m, m, m, m, m);
    
    return 0;
}