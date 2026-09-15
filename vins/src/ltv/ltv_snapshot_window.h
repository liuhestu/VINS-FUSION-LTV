#pragma once

#include "ltv_types.h"

#include <array>
#include <cstddef>

namespace ltv
{

template <std::size_t Capacity>
class LtvSnapshotWindow
{
  public:
    static_assert(Capacity >= 2, "an LTV snapshot window needs at least two slots");

    LtvSnapshotWindow()
    {
        clear();
    }

    void clear()
    {
        snapshots_.fill(LtvSnapshot{});
    }

    LtvSnapshot &operator[](std::size_t index)
    {
        return snapshots_[index];
    }

    const LtvSnapshot &operator[](std::size_t index) const
    {
        return snapshots_[index];
    }

    void slideOld()
    {
        for (std::size_t index = 0; index + 1 < Capacity; ++index)
            snapshots_[index] = snapshots_[index + 1];
        snapshots_[Capacity - 1] = snapshots_[Capacity - 2];
    }

    void slideSecondNewest()
    {
        snapshots_[Capacity - 2] = snapshots_[Capacity - 1];
    }

  private:
    std::array<LtvSnapshot, Capacity> snapshots_;
};

} // namespace ltv
