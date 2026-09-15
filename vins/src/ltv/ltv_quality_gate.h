#pragma once

#include "ltv_types.h"

namespace ltv
{

enum GravityGateReason : unsigned int
{
    GravityGateBaseIneligible = 1U << 0,
    GravityGateInsufficientFeatures = 1U << 1,
    GravityGateEtaNormMismatch = 1U << 2,
    GravityGateInnovationTooLarge = 1U << 3,
    GravityGateResetCooldown = 1U << 4
};

struct GravityGateDecision
{
    bool base_eligible = false;
    bool feature_ok = false;
    bool eta_norm_ok = false;
    bool innovation_ok = false;
    bool reset_ok = false;
    bool pass = false;
    unsigned int reason_mask = 0;
};

GravityGateDecision evaluateGravityQualityGate(const LtvConfig &config,
                                                const LtvSnapshot &snapshot,
                                                bool base_eligible,
                                                double gravity_norm_world,
                                                int cooldown_frames_remaining);

} // namespace ltv
