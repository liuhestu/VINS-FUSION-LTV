#include "utility/stereo_synchronizer.h"

#include <gtest/gtest.h>

TEST(StereoSynchronizer, MatchesAtToleranceBoundary)
{
    EXPECT_EQ(vins::classifyStereoTimestamps(10000000, 13000000),
              vins::StereoSyncDecision::Match);
    EXPECT_EQ(vins::classifyStereoTimestamps(13000000, 10000000),
              vins::StereoSyncDecision::Match);
}

TEST(StereoSynchronizer, DropsEarlierSideOutsideTolerance)
{
    EXPECT_EQ(vins::classifyStereoTimestamps(9999999, 13000000),
              vins::StereoSyncDecision::DropLeft);
    EXPECT_EQ(vins::classifyStereoTimestamps(13000000, 9999999),
              vins::StereoSyncDecision::DropRight);
}
