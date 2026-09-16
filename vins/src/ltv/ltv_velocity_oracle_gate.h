#pragma once

#include <eigen3/Eigen/Dense>

#include <cstdint>
#include <string>
#include <unordered_map>

namespace ltv
{

struct VelocityOracleGateDecision
{
    bool loaded = false;
    bool hit = false;
    bool pass = false;
};

class VelocityOracleGate
{
  public:
    bool configure(bool enabled, const std::string &mask_path,
                   const std::string &mask_column);
    VelocityOracleGateDecision evaluate(bool base_eligible,
                                        double frame_timestamp) const;

  private:
    bool enabled_ = false;
    bool loaded_ = false;
    std::unordered_map<std::int64_t, bool> mask_;
};

} // namespace ltv
