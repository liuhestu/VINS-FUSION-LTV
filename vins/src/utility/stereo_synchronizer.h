#pragma once

#include <cstdint>
#include <cstdlib>

namespace vins
{

constexpr std::int64_t kStereoToleranceNs = 3000000;

enum class StereoSyncDecision
{
    DropLeft,
    DropRight,
    Match
};

inline StereoSyncDecision classifyStereoTimestamps(
    std::int64_t left_timestamp_ns, std::int64_t right_timestamp_ns,
    std::int64_t tolerance_ns = kStereoToleranceNs)
{
    const std::int64_t delta = left_timestamp_ns - right_timestamp_ns;
    if (std::llabs(delta) <= tolerance_ns)
        return StereoSyncDecision::Match;
    return delta < 0 ? StereoSyncDecision::DropLeft
                     : StereoSyncDecision::DropRight;
}

} // namespace vins
