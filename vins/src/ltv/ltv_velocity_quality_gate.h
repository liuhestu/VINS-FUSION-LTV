#pragma once

#include "ltv_types.h"

namespace ltv
{

enum VelocityGateReason : unsigned int
{
    VelocityGateBaseIneligible = 1U << 0,
    VelocityGateInsufficientFeatures = 1U << 1,
    VelocityGateInnovationTooLarge = 1U << 2,
    VelocityGateDisagreementTooLarge = 1U << 3,
    VelocityGateResetCooldown = 1U << 4,
    VelocityGateInvalidConfiguration = 1U << 5,
    VelocityGateOracleConflict = 1U << 6
};

struct VelocityGateDecision
{
    bool base_eligible = false;
    bool feature_ok = false;
    bool innovation_ok = false;
    bool disagreement_ok = false;
    bool reset_ok = false;
    bool pass = false;
    unsigned int reason_mask = 0;
};

class VelocityQualityGate
{
  public:
    bool configure(const LtvConfig &config);
    VelocityGateDecision evaluate(const LtvSnapshot &snapshot,
                                  bool base_eligible,
                                  const Eigen::Vector3d &vins_velocity_body,
                                  int cooldown_frames_remaining) const;
    bool configurationValid() const;
    bool oracleConflict() const;

  private:
    LtvConfig config_;
    bool configuration_valid_ = true;
    bool oracle_conflict_ = false;
};

} // namespace ltv
