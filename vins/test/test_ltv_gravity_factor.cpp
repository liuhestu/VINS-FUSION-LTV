#include "factor/ltv_gravity_factor.h"
#include "utility/utility.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <random>

namespace
{

constexpr double kSigma = 10.0 * M_PI / 180.0;
const Eigen::Vector3d kGravityWorld(0.0, 0.0, -9.81);

std::array<double, 7> makePose(const Eigen::Quaterniond &rotation,
                               const Eigen::Vector3d &position = Eigen::Vector3d::Zero())
{
    return {{position.x(), position.y(), position.z(),
             rotation.x(), rotation.y(), rotation.z(), rotation.w()}};
}

Eigen::Vector3d evaluateResidual(const LtvGravityFactor &factor,
                                 const std::array<double, 7> &pose)
{
    const double *parameters[] = {pose.data()};
    Eigen::Vector3d residual;
    EXPECT_TRUE(factor.Evaluate(parameters, residual.data(), nullptr));
    return residual;
}

void expectJacobianMatchesFiniteDifference(const Eigen::Quaterniond &rotation)
{
    const Eigen::Vector3d gravity_body = rotation.inverse() * kGravityWorld;
    LtvGravityFactor factor(gravity_body, kGravityWorld, kSigma);
    std::array<double, 7> pose = makePose(rotation);
    const double *parameters[] = {pose.data()};
    Eigen::Vector3d residual;
    Eigen::Matrix<double, 3, 7, Eigen::RowMajor> analytic;
    double *jacobians[] = {analytic.data()};
    ASSERT_TRUE(factor.Evaluate(parameters, residual.data(), jacobians));

    Eigen::Matrix3d numerical;
    constexpr double epsilon = 1e-7;
    for (int axis = 0; axis < 3; ++axis)
    {
        Eigen::Vector3d delta = Eigen::Vector3d::Zero();
        delta(axis) = epsilon;
        const Eigen::Quaterniond plus_rotation =
            (rotation * Utility::deltaQ(delta)).normalized();
        const Eigen::Quaterniond minus_rotation =
            (rotation * Utility::deltaQ(-delta)).normalized();
        numerical.col(axis) =
            (evaluateResidual(factor, makePose(plus_rotation)) -
             evaluateResidual(factor, makePose(minus_rotation))) /
            (2.0 * epsilon);
    }

    EXPECT_LT((analytic.block<3, 3>(0, 3) - numerical).cwiseAbs().maxCoeff(), 1e-6);
    EXPECT_TRUE(analytic.leftCols<3>().isZero(0.0));
    EXPECT_TRUE(analytic.rightCols<1>().isZero(0.0));
}

TEST(LtvGravityFactor, MatchingDirectionsHaveZeroResidual)
{
    const Eigen::Quaterniond rotation(
        Eigen::AngleAxisd(0.7, Eigen::Vector3d(1.0, -2.0, 0.5).normalized()));
    const Eigen::Vector3d gravity_body = rotation.inverse() * kGravityWorld;
    const LtvGravityFactor factor(gravity_body, kGravityWorld, kSigma);

    EXPECT_TRUE(evaluateResidual(factor, makePose(rotation)).isZero(1e-12));
}

TEST(LtvGravityFactor, ResidualIgnoresGravityMagnitudeAndPosition)
{
    const Eigen::Quaterniond rotation(
        Eigen::AngleAxisd(-0.9, Eigen::Vector3d(0.3, 0.7, -0.2).normalized()));
    const Eigen::Vector3d gravity_body = 1.2 * (rotation.inverse() * kGravityWorld);
    const LtvGravityFactor factor(gravity_body, kGravityWorld, kSigma);

    EXPECT_TRUE(evaluateResidual(factor, makePose(rotation, Eigen::Vector3d(9.0, -3.0, 2.0)))
                    .isZero(1e-12));
}

TEST(LtvGravityFactor, OppositeDirectionsHaveExpectedNorm)
{
    const LtvGravityFactor factor(-kGravityWorld, kGravityWorld, kSigma);
    EXPECT_NEAR(evaluateResidual(factor, makePose(Eigen::Quaterniond::Identity())).norm(),
                2.0 / kSigma, 1e-12);
}

TEST(LtvGravityFactor, AnalyticJacobianMatchesRightPerturbation)
{
    expectJacobianMatchesFiniteDifference(Eigen::Quaterniond::Identity());
    expectJacobianMatchesFiniteDifference(Eigen::Quaterniond(
        Eigen::AngleAxisd(1.2, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(-0.8, Eigen::Vector3d::UnitY())));

    std::mt19937 generator(42);
    std::normal_distribution<double> normal(0.0, 1.0);
    for (int sample = 0; sample < 20; ++sample)
    {
        Eigen::Quaterniond rotation(normal(generator), normal(generator),
                                    normal(generator), normal(generator));
        rotation.normalize();
        expectJacobianMatchesFiniteDifference(rotation);
    }
}

TEST(LtvGravityFactor, RejectsInvalidConstructionInput)
{
    const LtvGravityFactor factor(Eigen::Vector3d::Zero(), kGravityWorld, kSigma);
    const std::array<double, 7> pose = makePose(Eigen::Quaterniond::Identity());
    const double *parameters[] = {pose.data()};
    Eigen::Vector3d residual;
    EXPECT_FALSE(factor.Evaluate(parameters, residual.data(), nullptr));
}

} // namespace
