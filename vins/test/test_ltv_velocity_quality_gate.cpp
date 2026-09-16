#include "ltv/ltv_velocity_quality_gate.h"

#include <gtest/gtest.h>

#include <limits>

namespace
{

ltv::LtvConfig testConfig()
{
    ltv::LtvConfig config;
    config.enable_velocity_quality_gate = true;
    config.max_features = 30;
    config.velocity_gate_min_features = 25;
    config.velocity_gate_max_normalized_innovation = 0.03;
    config.velocity_gate_max_disagreement_mps = 0.5;
    config.velocity_gate_reset_cooldown_frames = 10;
    return config;
}

ltv::LtvSnapshot validSnapshot()
{
    ltv::LtvSnapshot snapshot;
    snapshot.observed_features = 25;
    snapshot.innovation_norm = 0.15;
    snapshot.velocity_body = Eigen::Vector3d(0.5, 0.0, 0.0);
    snapshot.last_reset_reason = ltv::LtvResetReason::None;
    return snapshot;
}

TEST(LtvVelocityQualityGate, InclusiveThresholdsPass)
{
    ltv::VelocityQualityGate gate;
    ASSERT_TRUE(gate.configure(testConfig()));
    const ltv::VelocityGateDecision decision = gate.evaluate(
        validSnapshot(), true, Eigen::Vector3d::Zero(), 0);

    EXPECT_TRUE(decision.feature_ok);
    EXPECT_TRUE(decision.innovation_ok);
    EXPECT_TRUE(decision.disagreement_ok);
    EXPECT_TRUE(decision.reset_ok);
    EXPECT_TRUE(decision.pass);
    EXPECT_EQ(decision.reason_mask, 0U);
}

TEST(LtvVelocityQualityGate, ReportsIndependentAndCombinedRejections)
{
    ltv::VelocityQualityGate gate;
    ASSERT_TRUE(gate.configure(testConfig()));

    ltv::LtvSnapshot feature = validSnapshot();
    feature.observed_features = 24;
    feature.innovation_norm = 0.0;
    EXPECT_EQ(gate.evaluate(feature, true, feature.velocity_body, 0).reason_mask,
              ltv::VelocityGateInsufficientFeatures);

    ltv::LtvSnapshot innovation = validSnapshot();
    innovation.innovation_norm = 0.16;
    EXPECT_EQ(gate.evaluate(innovation, true, innovation.velocity_body, 0).reason_mask,
              ltv::VelocityGateInnovationTooLarge);

    EXPECT_EQ(gate.evaluate(validSnapshot(), true,
                            Eigen::Vector3d(-0.01, 0.0, 0.0), 0).reason_mask,
              ltv::VelocityGateDisagreementTooLarge);
    EXPECT_EQ(gate.evaluate(validSnapshot(), true,
                            validSnapshot().velocity_body, 1).reason_mask,
              ltv::VelocityGateResetCooldown);

    ltv::LtvSnapshot combined = validSnapshot();
    combined.observed_features = 1;
    combined.innovation_norm = 10.0;
    combined.last_reset_reason = ltv::LtvResetReason::TimestampGap;
    const ltv::VelocityGateDecision rejected = gate.evaluate(
        combined, false, Eigen::Vector3d(-1.0, 0.0, 0.0), 10);
    EXPECT_FALSE(rejected.pass);
    EXPECT_NE(rejected.reason_mask & ltv::VelocityGateBaseIneligible, 0U);
    EXPECT_NE(rejected.reason_mask & ltv::VelocityGateInsufficientFeatures, 0U);
    EXPECT_NE(rejected.reason_mask & ltv::VelocityGateInnovationTooLarge, 0U);
    EXPECT_NE(rejected.reason_mask & ltv::VelocityGateDisagreementTooLarge, 0U);
    EXPECT_NE(rejected.reason_mask & ltv::VelocityGateResetCooldown, 0U);
}

TEST(LtvVelocityQualityGate, NanAndInfinityFailClosed)
{
    ltv::VelocityQualityGate gate;
    ASSERT_TRUE(gate.configure(testConfig()));
    ltv::LtvSnapshot snapshot = validSnapshot();
    snapshot.innovation_norm = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(gate.evaluate(
        snapshot, true, snapshot.velocity_body, 0).innovation_ok);

    snapshot = validSnapshot();
    snapshot.velocity_body.x() = std::numeric_limits<double>::infinity();
    const ltv::VelocityGateDecision decision = gate.evaluate(
        snapshot, true, Eigen::Vector3d::Zero(), 0);
    EXPECT_FALSE(decision.disagreement_ok);
    EXPECT_FALSE(decision.pass);
}

TEST(LtvVelocityQualityGate, ResetRejectsExactlyTenFollowingHealthyFrames)
{
    const ltv::LtvConfig config = testConfig();
    ltv::VelocityQualityGate gate;
    ASSERT_TRUE(gate.configure(config));
    ltv::LtvSnapshot snapshot = validSnapshot();
    int remaining = 0;

    snapshot.last_reset_reason = ltv::LtvResetReason::TimestampGap;
    remaining = config.velocity_gate_reset_cooldown_frames;
    EXPECT_FALSE(gate.evaluate(
        snapshot, false, snapshot.velocity_body, remaining).reset_ok);

    snapshot.last_reset_reason = ltv::LtvResetReason::None;
    for (int frame = 0; frame < 10; ++frame)
    {
        EXPECT_FALSE(gate.evaluate(
            snapshot, true, snapshot.velocity_body, remaining).reset_ok)
            << "healthy frame " << frame + 1;
        --remaining;
    }
    EXPECT_TRUE(gate.evaluate(
        snapshot, true, snapshot.velocity_body, remaining).pass);
}

TEST(LtvVelocityQualityGate, ConflictAndInvalidConfigurationFailClosed)
{
    ltv::LtvConfig conflict = testConfig();
    conflict.enable_velocity_oracle_gate = true;
    ltv::VelocityQualityGate conflict_gate;
    EXPECT_FALSE(conflict_gate.configure(conflict));
    const ltv::VelocityGateDecision conflict_decision = conflict_gate.evaluate(
        validSnapshot(), true, validSnapshot().velocity_body, 0);
    EXPECT_FALSE(conflict_decision.pass);
    EXPECT_NE(conflict_decision.reason_mask & ltv::VelocityGateOracleConflict, 0U);

    ltv::LtvConfig invalid = testConfig();
    invalid.velocity_gate_max_disagreement_mps = -0.1;
    ltv::VelocityQualityGate invalid_gate;
    EXPECT_FALSE(invalid_gate.configure(invalid));
    const ltv::VelocityGateDecision invalid_decision = invalid_gate.evaluate(
        validSnapshot(), true, validSnapshot().velocity_body, 0);
    EXPECT_FALSE(invalid_decision.pass);
    EXPECT_NE(invalid_decision.reason_mask &
              ltv::VelocityGateInvalidConfiguration, 0U);

    invalid = testConfig();
    invalid.velocity_gate_max_normalized_innovation =
        std::numeric_limits<double>::infinity();
    EXPECT_FALSE(invalid_gate.configure(invalid));
    EXPECT_FALSE(invalid_gate.evaluate(
        validSnapshot(), true, validSnapshot().velocity_body, 0).pass);
}

} // namespace
