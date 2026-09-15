#include "ltv/ltv_quality_gate.h"

#include <gtest/gtest.h>

namespace
{

ltv::LtvConfig testConfig()
{
    ltv::LtvConfig config;
    config.gravity_gate_min_features = 5;
    config.gravity_gate_max_eta_norm_error = 0.5;
    config.gravity_gate_max_normalized_innovation = 1.0;
    return config;
}

ltv::LtvSnapshot validSnapshot()
{
    ltv::LtvSnapshot snapshot;
    snapshot.observed_features = 9;
    snapshot.gravity_body = Eigen::Vector3d(0.0, 0.0, 9.81);
    snapshot.innovation_norm = 2.0;
    snapshot.last_reset_reason = ltv::LtvResetReason::None;
    return snapshot;
}

TEST(LtvGravityQualityGate, AcceptsHealthySnapshot)
{
    const ltv::GravityGateDecision decision = ltv::evaluateGravityQualityGate(
        testConfig(), validSnapshot(), true, 9.81, 0);

    EXPECT_TRUE(decision.pass);
    EXPECT_EQ(decision.reason_mask, 0U);
}

TEST(LtvGravityQualityGate, ReportsEveryRejectedCondition)
{
    ltv::LtvSnapshot snapshot = validSnapshot();
    snapshot.observed_features = 2;
    snapshot.gravity_body = Eigen::Vector3d(0.0, 0.0, 8.0);
    snapshot.innovation_norm = 10.0;
    snapshot.last_reset_reason = ltv::LtvResetReason::TimestampGap;

    const ltv::GravityGateDecision decision = ltv::evaluateGravityQualityGate(
        testConfig(), snapshot, false, 9.81, 2);

    EXPECT_FALSE(decision.pass);
    EXPECT_NE(decision.reason_mask & ltv::GravityGateBaseIneligible, 0U);
    EXPECT_NE(decision.reason_mask & ltv::GravityGateInsufficientFeatures, 0U);
    EXPECT_NE(decision.reason_mask & ltv::GravityGateEtaNormMismatch, 0U);
    EXPECT_NE(decision.reason_mask & ltv::GravityGateInnovationTooLarge, 0U);
    EXPECT_NE(decision.reason_mask & ltv::GravityGateResetCooldown, 0U);
}

TEST(LtvGravityQualityGate, DisabledInnovationThresholdAlwaysPassesInnovation)
{
    ltv::LtvConfig config = testConfig();
    config.gravity_gate_max_normalized_innovation = -1.0;
    ltv::LtvSnapshot snapshot = validSnapshot();
    snapshot.innovation_norm = 1.0e12;

    const ltv::GravityGateDecision decision = ltv::evaluateGravityQualityGate(
        config, snapshot, true, 9.81, 0);

    EXPECT_TRUE(decision.innovation_ok);
    EXPECT_TRUE(decision.pass);
}

} // namespace
