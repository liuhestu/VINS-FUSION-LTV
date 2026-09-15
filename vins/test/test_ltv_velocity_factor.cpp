#include "factor/ltv_velocity_factor.h"
#include "factor/pose_local_parameterization.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <random>

namespace
{

constexpr double kSigma = 1.0;

std::array<double, 7> makePose(const Eigen::Quaterniond &rotation,
                               const Eigen::Vector3d &position = Eigen::Vector3d::Zero())
{
    return {{position.x(), position.y(), position.z(), rotation.x(), rotation.y(),
             rotation.z(), rotation.w()}};
}

std::array<double, 9> makeSpeedBias(const Eigen::Vector3d &velocity)
{
    return {{velocity.x(), velocity.y(), velocity.z(), 0.2, -0.1, 0.3, -0.02, 0.01, -0.03}};
}

Eigen::Vector3d evaluateResidual(const LtvVelocityFactor &factor,
                                 const std::array<double, 7> &pose,
                                 const std::array<double, 9> &speed_bias)
{
    const double *parameters[] = {pose.data(), speed_bias.data()};
    Eigen::Vector3d residual;
    EXPECT_TRUE(factor.Evaluate(parameters, residual.data(), nullptr));
    return residual;
}

void expectJacobiansMatchFiniteDifference(const Eigen::Quaterniond &rotation,
                                          const Eigen::Vector3d &velocity_world)
{
    const Eigen::Vector3d velocity_body = rotation.inverse() * velocity_world +
                                          Eigen::Vector3d(0.1, -0.2, 0.05);
    const LtvVelocityFactor factor(velocity_body, kSigma);
    std::array<double, 7> pose = makePose(rotation, Eigen::Vector3d(4.0, -3.0, 2.0));
    std::array<double, 9> speed_bias = makeSpeedBias(velocity_world);
    const double *parameters[] = {pose.data(), speed_bias.data()};

    Eigen::Vector3d residual;
    Eigen::Matrix<double, 3, 7, Eigen::RowMajor> pose_ambient;
    Eigen::Matrix<double, 3, 9, Eigen::RowMajor> speed_bias_analytic;
    double *jacobians[] = {pose_ambient.data(), speed_bias_analytic.data()};
    ASSERT_TRUE(factor.Evaluate(parameters, residual.data(), jacobians));

    PoseLocalParameterization pose_parameterization;
    Eigen::Matrix<double, 7, 6, Eigen::RowMajor> plus_jacobian;
    ASSERT_TRUE(pose_parameterization.PlusJacobian(pose.data(), plus_jacobian.data()));
    const Eigen::Matrix<double, 3, 6> pose_effective = pose_ambient * plus_jacobian;

    constexpr double epsilon = 1e-6;
    Eigen::Matrix<double, 3, 6> pose_numerical;
    for (int axis = 0; axis < 6; ++axis)
    {
        Eigen::Matrix<double, 6, 1> delta = Eigen::Matrix<double, 6, 1>::Zero();
        std::array<double, 7> plus_pose;
        std::array<double, 7> minus_pose;
        delta(axis) = epsilon;
        ASSERT_TRUE(pose_parameterization.Plus(pose.data(), delta.data(), plus_pose.data()));
        delta(axis) = -epsilon;
        ASSERT_TRUE(pose_parameterization.Plus(pose.data(), delta.data(), minus_pose.data()));
        pose_numerical.col(axis) =
            (evaluateResidual(factor, plus_pose, speed_bias) -
             evaluateResidual(factor, minus_pose, speed_bias)) / (2.0 * epsilon);
    }
    EXPECT_LT((pose_effective - pose_numerical).cwiseAbs().maxCoeff(), 1e-6);
    EXPECT_TRUE(pose_effective.leftCols<3>().isZero(0.0));

    Eigen::Matrix<double, 3, 9> speed_bias_numerical;
    for (int axis = 0; axis < 9; ++axis)
    {
        std::array<double, 9> plus_speed_bias = speed_bias;
        std::array<double, 9> minus_speed_bias = speed_bias;
        plus_speed_bias[axis] += epsilon;
        minus_speed_bias[axis] -= epsilon;
        speed_bias_numerical.col(axis) =
            (evaluateResidual(factor, pose, plus_speed_bias) -
             evaluateResidual(factor, pose, minus_speed_bias)) / (2.0 * epsilon);
    }
    EXPECT_LT((speed_bias_analytic - speed_bias_numerical).cwiseAbs().maxCoeff(), 1e-6);
    EXPECT_TRUE(speed_bias_analytic.rightCols<6>().isZero(0.0));
}

TEST(LtvVelocityFactor, MatchingVelocityHasZeroResidual)
{
    const Eigen::Quaterniond rotation(Eigen::AngleAxisd(
        0.7, Eigen::Vector3d(1.0, -2.0, 0.5).normalized()));
    const Eigen::Vector3d velocity_world(0.8, -0.4, 0.3);
    const LtvVelocityFactor factor(rotation.inverse() * velocity_world, kSigma);

    EXPECT_TRUE(evaluateResidual(factor, makePose(rotation), makeSpeedBias(velocity_world))
                    .isZero(1e-12));
}

TEST(LtvVelocityFactor, RotationTransformsWorldVelocityIntoBodyFrame)
{
    const Eigen::Quaterniond rotation(Eigen::AngleAxisd(
        M_PI_2, Eigen::Vector3d::UnitZ()));
    const Eigen::Vector3d velocity_world(1.0, 0.0, 0.0);
    const LtvVelocityFactor factor(Eigen::Vector3d(0.0, -1.0, 0.0), kSigma);

    EXPECT_TRUE(evaluateResidual(factor, makePose(rotation), makeSpeedBias(velocity_world))
                    .isZero(1e-12));
}

TEST(LtvVelocityFactor, ResidualIgnoresPositionAndBiases)
{
    const Eigen::Quaterniond rotation(Eigen::AngleAxisd(
        -0.6, Eigen::Vector3d(0.2, 0.7, -0.4).normalized()));
    const Eigen::Vector3d velocity_world(0.7, 0.5, -0.2);
    const LtvVelocityFactor factor(rotation.inverse() * velocity_world, kSigma);
    const std::array<double, 7> pose = makePose(rotation, Eigen::Vector3d(5.0, -4.0, 3.0));
    std::array<double, 9> speed_bias = makeSpeedBias(velocity_world);
    speed_bias[3] = 10.0;
    speed_bias[4] = -8.0;
    speed_bias[5] = 6.0;
    speed_bias[6] = -4.0;
    speed_bias[7] = 2.0;
    speed_bias[8] = -1.0;

    EXPECT_TRUE(evaluateResidual(factor, pose, speed_bias).isZero(1e-12));
}

TEST(LtvVelocityFactor, AnalyticJacobiansMatchEffectiveLocalJacobians)
{
    expectJacobiansMatchFiniteDifference(Eigen::Quaterniond::Identity(),
                                         Eigen::Vector3d(0.4, -0.8, 0.2));
    expectJacobiansMatchFiniteDifference(
        Eigen::Quaterniond(Eigen::AngleAxisd(1.2, Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(-0.8, Eigen::Vector3d::UnitY())),
        Eigen::Vector3d(0.7, 0.2, -0.5));

    std::mt19937 generator(42);
    std::normal_distribution<double> normal(0.0, 1.0);
    for (int sample = 0; sample < 20; ++sample)
    {
        Eigen::Quaterniond rotation(normal(generator), normal(generator),
                                    normal(generator), normal(generator));
        rotation.normalize();
        expectJacobiansMatchFiniteDifference(
            rotation, Eigen::Vector3d(normal(generator), normal(generator), normal(generator)));
    }
}

TEST(LtvVelocityFactor, RejectsInvalidInput)
{
    const Eigen::Vector3d invalid_velocity(
        std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
    const LtvVelocityFactor invalid_factor(invalid_velocity, kSigma);
    const std::array<double, 7> pose = makePose(Eigen::Quaterniond::Identity());
    const std::array<double, 9> speed_bias = makeSpeedBias(Eigen::Vector3d::Zero());
    const double *parameters[] = {pose.data(), speed_bias.data()};
    Eigen::Vector3d residual;
    EXPECT_FALSE(invalid_factor.Evaluate(parameters, residual.data(), nullptr));

    const LtvVelocityFactor zero_sigma_factor(Eigen::Vector3d::Zero(), 0.0);
    EXPECT_FALSE(zero_sigma_factor.Evaluate(parameters, residual.data(), nullptr));
}

} // namespace
