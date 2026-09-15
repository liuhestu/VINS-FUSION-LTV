#include "ltv_quality_gate.h"

#include <algorithm>
#include <cmath>

namespace ltv
{

GravityGateDecision evaluateGravityQualityGate(const LtvConfig &config,
                                                const LtvSnapshot &snapshot,
                                                bool base_eligible,
                                                double gravity_norm_world,
                                                int cooldown_frames_remaining)
{
    GravityGateDecision decision;
    decision.base_eligible = base_eligible;
    decision.feature_ok = snapshot.observed_features >= config.gravity_gate_min_features;

    const double eta_norm = snapshot.gravity_body.norm();
    decision.eta_norm_ok = std::isfinite(eta_norm) &&
                           std::isfinite(gravity_norm_world) &&
                           std::abs(eta_norm - gravity_norm_world) <=
                               config.gravity_gate_max_eta_norm_error;

    if (config.gravity_gate_max_normalized_innovation < 0.0)
    {
        decision.innovation_ok = true;
    }
    else
    {
        const double normalization = std::sqrt(
            static_cast<double>(std::max(1, snapshot.observed_features)));
        const double normalized_innovation = snapshot.innovation_norm / normalization;
        decision.innovation_ok = std::isfinite(normalized_innovation) &&
                                 normalized_innovation <=
                                     config.gravity_gate_max_normalized_innovation;
    }

    decision.reset_ok = snapshot.last_reset_reason == LtvResetReason::None &&
                        cooldown_frames_remaining <= 0;
    decision.pass = decision.base_eligible && decision.feature_ok &&
                    decision.eta_norm_ok && decision.innovation_ok &&
                    decision.reset_ok;

    if (!decision.base_eligible)
        decision.reason_mask |= GravityGateBaseIneligible;
    if (!decision.feature_ok)
        decision.reason_mask |= GravityGateInsufficientFeatures;
    if (!decision.eta_norm_ok)
        decision.reason_mask |= GravityGateEtaNormMismatch;
    if (!decision.innovation_ok)
        decision.reason_mask |= GravityGateInnovationTooLarge;
    if (!decision.reset_ok)
        decision.reason_mask |= GravityGateResetCooldown;
    return decision;
}

} // namespace ltv
