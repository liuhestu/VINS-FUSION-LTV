#include "ltv/ltv_snapshot_window.h"

#include <gtest/gtest.h>

namespace
{

constexpr std::size_t kWindowCapacity = 11;

ltv::LtvSnapshot makeSnapshot(int frame_id)
{
    ltv::LtvSnapshot snapshot;
    snapshot.frame_timestamp = static_cast<double>(frame_id);
    snapshot.gravity_body.x() = static_cast<double>(frame_id);
    snapshot.valid = true;
    snapshot.gravity_valid = true;
    return snapshot;
}

void expectFrame(const ltv::LtvSnapshot &snapshot, int frame_id)
{
    EXPECT_DOUBLE_EQ(snapshot.frame_timestamp, static_cast<double>(frame_id));
    EXPECT_DOUBLE_EQ(snapshot.gravity_body.x(), static_cast<double>(frame_id));
    EXPECT_TRUE(snapshot.valid);
    EXPECT_TRUE(snapshot.gravity_valid);
}

TEST(LtvSnapshotWindow, StartsAndClearsInvalid)
{
    ltv::LtvSnapshotWindow<kWindowCapacity> window;

    for (std::size_t index = 0; index < kWindowCapacity; ++index)
    {
        EXPECT_FALSE(window[index].valid);
        window[index] = makeSnapshot(static_cast<int>(index));
    }

    window.clear();
    for (std::size_t index = 0; index < kWindowCapacity; ++index)
    {
        EXPECT_FALSE(window[index].valid);
        EXPECT_FALSE(window[index].gravity_valid);
        EXPECT_DOUBLE_EQ(window[index].frame_timestamp, 0.0);
    }
}

TEST(LtvSnapshotWindow, MarginOldMatchesEstimatorPermutation)
{
    ltv::LtvSnapshotWindow<kWindowCapacity> window;
    for (std::size_t index = 0; index < kWindowCapacity; ++index)
        window[index] = makeSnapshot(static_cast<int>(index));

    window.slideOld();

    for (std::size_t index = 0; index + 1 < kWindowCapacity; ++index)
        expectFrame(window[index], static_cast<int>(index + 1));
    expectFrame(window[kWindowCapacity - 1], static_cast<int>(kWindowCapacity - 1));
}

TEST(LtvSnapshotWindow, MarginSecondNewestMatchesEstimatorPermutation)
{
    ltv::LtvSnapshotWindow<kWindowCapacity> window;
    for (std::size_t index = 0; index < kWindowCapacity; ++index)
        window[index] = makeSnapshot(static_cast<int>(index));

    window.slideSecondNewest();

    for (std::size_t index = 0; index + 2 < kWindowCapacity; ++index)
        expectFrame(window[index], static_cast<int>(index));
    expectFrame(window[kWindowCapacity - 2], static_cast<int>(kWindowCapacity - 1));
    expectFrame(window[kWindowCapacity - 1], static_cast<int>(kWindowCapacity - 1));
}

} // namespace
