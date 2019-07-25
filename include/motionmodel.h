#ifndef MOTIONMODEL_H
#define MOTIONMODEL_H

#include "Eigen/Dense"
#include "gaussian.h"


class MotionModel {
    public:
        MotionModel();
        ~MotionModel();
        
        Gaussian CalculateMotion(
            Eigen::Vector3f p_odometry, 
            Gaussian p_gaussian,
            Eigen::Matrix3f p_motionNoise
        );
    private:
};



#endif