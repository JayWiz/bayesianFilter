#include "gtest/gtest.h"
#include "motionmodel.h"

TEST(motionmodel, calculatemotion) {
    MotionModel m1;

    Eigen::Vector3f mu0, mu1, mu2, mu1_test, mu2_test;
    mu1_test << 0, 3, 0;
    mu2_test << 2, 3, 90;
    mu0 << 0, 0, 0;

    Eigen::Vector3f odom1, odom2;
    odom1 << 1.5708, -1.5708, 3;
    odom2 << 0, 90, 2;

    Eigen::Matrix3f cov0, cov1, cov, cov1_test, cov2_test;
    cov1_test << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;
    cov2_test << 0.2, 0, 0, 0, 0.6, 0.2, 0, 0.2, 0.2;
    cov0 << 0, 0, 0, 0, 0, 0, 0, 0, 0;

    Eigen::Matrix3f Q;
    Q << 0.1, 0, 0, 0, 0.1, 0, 0, 0, 0.1;

    Gaussian g0 = Gaussian(cov0, mu0);
    Gaussian g1 = m1.CalculateMotion(odom1, g0, Q);
    Gaussian g2 = m1.CalculateMotion(odom2, g1, Q);
    
    EXPECT_TRUE(g1.GetCovariance().isApprox(cov1_test)) << g1.GetCovariance();
    EXPECT_TRUE(g1.GetExpectedValue().isApprox(mu1_test, 1e-4)) << g1.GetExpectedValue();
    EXPECT_TRUE(g2.GetCovariance().isApprox(cov2_test)) << g2.GetCovariance();
    EXPECT_TRUE(g2.GetExpectedValue().isApprox(mu2_test, 1e-4)) << g2.GetExpectedValue();
}