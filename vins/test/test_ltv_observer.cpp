#include "ltv/ltv_csv_logger.h"
#include "ltv/ltv_observer.h"

#include <gtest/gtest.h>

namespace
{

ltv::LtvConfig testConfig()
{
    ltv::LtvConfig config;
    config.enable = true;
    config.log_debug = false;
    config.max_features = 30;
    config.min_features = 1;
    config.max_missed_frames = 0;
    config.warmup_camera_updates = 1;
    config.max_imu_dt = 0.1;
    config.reset_gap = 0.2;
    config.timestamp_tolerance = 1e-9;
    config.q_landmark = 0.0;
    config.v_landmark = 0.0;
    config.v_velocity = 0.0;
    config.v_gravity = 0.0;
    config.covariance_failure_threshold = -1.0;
    return config;
}

ltv::LtvFeatureObservation observation(int id, const Eigen::Vector3d &coordinate)
{
    ltv::LtvFeatureObservation result;
    result.feature_id = id;
    result.normalized_coordinate = coordinate;
    return result;
}

TEST(LtvProjection, IsSymmetricIdempotentAndOrthogonal)
{
    const Eigen::AngleAxisd rotation(0.4, Eigen::Vector3d(1.0, 2.0, 3.0).normalized());
    const Eigen::Vector3d bearing_camera = Eigen::Vector3d(0.2, -0.3, 1.0).normalized();
    const Eigen::Vector3d bearing_body = rotation.toRotationMatrix() * bearing_camera;
    const Eigen::Matrix3d projection =
        Eigen::Matrix3d::Identity() - bearing_body * bearing_body.transpose();

    EXPECT_TRUE(projection.isApprox(projection.transpose(), 1e-12));
    EXPECT_TRUE((projection * projection).isApprox(projection, 1e-12));
    EXPECT_LT((projection * bearing_body).norm(), 1e-12);
}

TEST(LtvObserver, StateDimensionsFollowFeatureCount)
{
    ltv::LtvObserver observer;
    observer.configure(testConfig());
    observer.start(0.0);

    std::vector<ltv::LtvFeatureObservation> observations;
    for (int id = 0; id < 16; ++id)
        observations.push_back(observation(id, Eigen::Vector3d(0.01 * id, 0.02, 1.0)));
    observer.updateFeatures(0.0, 0.0, observations,
                            Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

    EXPECT_EQ(observer.state().size(), 3 * 16 + 6);
    EXPECT_EQ(observer.covariance().rows(), 3 * 16 + 6);
    EXPECT_EQ(observer.covariance().cols(), 3 * 16 + 6);
}

TEST(LtvObserver, ImuPropagationUsesBodyFrameApparentAcceleration)
{
    ltv::LtvObserver observer;
    observer.configure(testConfig());
    observer.start(1.0);
    observer.updateFeatures(1.0, 1.0,
                            {observation(7, Eigen::Vector3d(0.0, 0.0, 1.0))},
                            Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

    const Eigen::Vector3d measured_acceleration(1.0, -2.0, 3.0);
    const Eigen::Vector3d accel_bias(0.1, 0.2, 0.3);
    observer.propagateImu(0.01, measured_acceleration, Eigen::Vector3d::Zero(),
                          accel_bias, Eigen::Vector3d::Zero());

    const int velocity_offset = 3;
    EXPECT_TRUE(observer.state().segment<3>(velocity_offset).isApprox(
        0.01 * (measured_acceleration - accel_bias), 1e-12));
    EXPECT_TRUE(observer.state().tail<3>().isZero(1e-12));
}

TEST(LtvObserver, FeatureCompactionPreservesIdentity)
{
    ltv::LtvObserver observer;
    ltv::LtvConfig config = testConfig();
    config.max_features = 2;
    observer.configure(config);
    observer.start(2.0);
    observer.updateFeatures(
        2.0, 2.0,
        {observation(10, Eigen::Vector3d(0.0, 0.0, 1.0)),
         observation(20, Eigen::Vector3d(0.1, 0.0, 1.0))},
        Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

    ASSERT_GE(observer.slotForFeature(10), 0);
    ASSERT_GE(observer.slotForFeature(20), 0);
    observer.propagateImu(0.01, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                          Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    observer.updateFeatures(
        2.01, 2.01, {observation(20, Eigen::Vector3d(0.1, 0.0, 1.0))},
        Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

    EXPECT_EQ(observer.slotForFeature(10), -1);
    EXPECT_EQ(observer.slotForFeature(20), 0);
    EXPECT_EQ(observer.state().size(), 9);
}

TEST(LtvObserver, TimestampBackwardResetsObserver)
{
    ltv::LtvObserver observer;
    observer.configure(testConfig());
    observer.start(3.0);
    observer.updateFeatures(3.0, 3.0,
                            {observation(1, Eigen::Vector3d(0.0, 0.0, 1.0))},
                            Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
    const ltv::LtvSnapshot result = observer.updateFeatures(
        2.9, 2.9, {observation(1, Eigen::Vector3d(0.0, 0.0, 1.0))},
        Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

    EXPECT_FALSE(observer.started());
    EXPECT_FALSE(result.valid);
    EXPECT_EQ(result.last_reset_reason, ltv::LtvResetReason::TimestampBackward);
}

TEST(LtvObserver, AdaptiveCameraSubstepsPreserveProductionScaleCovariance)
{
    ltv::LtvConfig config = testConfig();
    config.max_features = 30;
    config.min_features = 15;
    config.q_landmark = 1e-4;
    config.v_landmark = 1e6;
    config.v_velocity = 1e6;
    config.v_gravity = 1e6;
    config.covariance_failure_threshold = -1e-3;

    std::vector<ltv::LtvFeatureObservation> observations;
    for (int id = 0; id < 30; ++id)
        observations.push_back(observation(id, Eigen::Vector3d(0.01 * id, -0.1, 1.0)));

    ltv::LtvObserver observer;
    observer.configure(config);
    observer.start(4.0);
    observer.updateFeatures(4.0, 4.0, observations,
                            Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.02, 0.06, 0.01));
    for (int index = 0; index < 20; ++index)
    {
        observer.propagateImu(0.005, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                              Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    }
    const ltv::LtvSnapshot result = observer.updateFeatures(
        4.1, 4.1, observations, Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.02, 0.06, 0.01));

    EXPECT_TRUE(observer.started());
    EXPECT_GT(result.camera_substeps, 1);
    EXPECT_TRUE(observer.covariance().allFinite());
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(observer.covariance());
    ASSERT_EQ(solver.info(), Eigen::Success);
    EXPECT_GT(solver.eigenvalues().minCoeff(), 0.0);
}

TEST(LtvCsvLogger, DisabledLoggerDoesNotOpenFile)
{
    ltv::LtvCsvLogger logger;
    logger.configure(false, "/tmp/ltv_observer_disabled.csv");
    EXPECT_FALSE(logger.isOpen());
}

} // namespace
