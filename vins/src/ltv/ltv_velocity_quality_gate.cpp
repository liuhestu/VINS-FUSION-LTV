#include "ltv_velocity_quality_gate.h"

#include <algorithm>
#include <cmath>

namespace ltv
{

bool VelocityQualityGate::configure(const LtvConfig &config)
{
    config_ = config;
    oracle_conflict_ = config.enable_velocity_quality_gate &&
                       config.enable_velocity_oracle_gate;
    configuration_valid_ = !config.enable_velocity_quality_gate ||
        (config.velocity_gate_min_features > 0 &&
         config.velocity_gate_min_features <= config.max_features &&
         std::isfinite(config.velocity_gate_max_normalized_innovation) &&
         config.velocity_gate_max_normalized_innovation >= 0.0 &&
         std::isfinite(config.velocity_gate_max_disagreement_mps) &&
         config.velocity_gate_max_disagreement_mps >= 0.0 &&
         config.velocity_gate_reset_cooldown_frames >= 0);
    return configuration_valid_ && !oracle_conflict_;
}

VelocityGateDecision VelocityQualityGate::evaluate(
    const LtvSnapshot &snapshot, bool base_eligible,
    const Eigen::Vector3d &vins_velocity_body,
    int cooldown_frames_remaining) const
{
    VelocityGateDecision decision;
    decision.base_eligible = base_eligible;
    decision.feature_ok =
        snapshot.observed_features >= config_.velocity_gate_min_features;

    const double normalization = std::sqrt(
        static_cast<double>(std::max(1, snapshot.observed_features)));
    const double normalized_innovation = snapshot.innovation_norm / normalization;
    decision.innovation_ok = std::isfinite(normalized_innovation) &&
        normalized_innovation <= config_.velocity_gate_max_normalized_innovation;

    const double disagreement =
        (snapshot.velocity_body - vins_velocity_body).norm();
    decision.disagreement_ok = snapshot.velocity_body.allFinite() &&
        vins_velocity_body.allFinite() && std::isfinite(disagreement) &&
        disagreement <= config_.velocity_gate_max_disagreement_mps;
    decision.reset_ok = snapshot.last_reset_reason == LtvResetReason::None &&
                        cooldown_frames_remaining <= 0;
    decision.pass = configuration_valid_ && !oracle_conflict_ &&
                    decision.base_eligible && decision.feature_ok &&
                    decision.innovation_ok && decision.disagreement_ok &&
                    decision.reset_ok;

    if (!decision.base_eligible)
        decision.reason_mask |= VelocityGateBaseIneligible;
    if (!decision.feature_ok)
        decision.reason_mask |= VelocityGateInsufficientFeatures;
    if (!decision.innovation_ok)
        decision.reason_mask |= VelocityGateInnovationTooLarge;
    if (!decision.disagreement_ok)
        decision.reason_mask |= VelocityGateDisagreementTooLarge;
    if (!decision.reset_ok)
        decision.reason_mask |= VelocityGateResetCooldown;
    if (!configuration_valid_)
        decision.reason_mask |= VelocityGateInvalidConfiguration;
    if (oracle_conflict_)
        decision.reason_mask |= VelocityGateOracleConflict;
    return decision;
}

bool VelocityQualityGate::configurationValid() const
{
    return configuration_valid_;
}

bool VelocityQualityGate::oracleConflict() const
{
    return oracle_conflict_;
}

} // namespace ltv
