#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE "MotionModelModule"

#include <boost/test/unit_test.hpp>

#include "../include/motionmodel.h"
#include "../include/gaussian.h"

// TODO: Test Constructors

BOOST_AUTO_TEST_CASE(calculateMotion) {
    MotionModel m1;
    
    Eigen::Vector3f odom1, mu1, mu2;
    odom1 << 0, 0, 2;
    mu1 << 0, 0, 0;
    mu2 << 2, 0, 0;
    
    Eigen::Matrix3f cov1, cov2, qno1;
    cov1 << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    cov2 << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    qno1 << 1, 0, 0, 0, 1, 0, 0, 0, 1;

    Gaussian gaus1 = Gaussian(cov1, mu1);
    
    Gaussian gaus2 = m1.CalculateMotion(odom1, gaus1, qno1);

    Gaussian testGaus2 = Gaussian(cov2, mu2); 

    BOOST_CHECK_EQUAL(gaus2.GetCovariance(), testGaus2.GetCovariance());
}

