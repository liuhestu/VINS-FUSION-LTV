#include <gtest/gtest.h>

#include <cstdio>
#include <opencv2/imgcodecs.hpp>
#include <unistd.h>

#include "estimator/parameters.h"
#include "featureTracker/feature_tracker.h"

namespace
{

std::string temporaryPath(const char *suffix)
{
    return std::string("/tmp/vins_feature_mask_") + std::to_string(::getpid()) + suffix;
}

TEST(FeatureMasks, FiltersExistingLeftTracks)
{
    USE_MASK = 1;
    MIN_DIST = 1;
    MASK0 = cv::Mat(10, 10, CV_8UC1, cv::Scalar(255));
    MASK0.at<uchar>(2, 2) = 0;

    FeatureTracker tracker;
    tracker.row = 10;
    tracker.col = 10;
    tracker.cur_pts = {{2.0F, 2.0F}, {7.0F, 7.0F}};
    tracker.ids = {10, 11};
    tracker.track_cnt = {5, 4};
    tracker.setMask();

    ASSERT_EQ(tracker.ids.size(), 1U);
    EXPECT_EQ(tracker.ids.front(), 11);
    EXPECT_EQ(tracker.mask.at<uchar>(2, 2), 0);
}

TEST(FeatureMasks, DisabledPreservesCompatibility)
{
    USE_MASK = 0;
    MIN_DIST = 1;
    MASK0.release();

    FeatureTracker tracker;
    tracker.row = 10;
    tracker.col = 10;
    tracker.cur_pts = {{2.0F, 2.0F}, {7.0F, 7.0F}};
    tracker.ids = {10, 11};
    tracker.track_cnt = {5, 4};
    tracker.setMask();

    EXPECT_EQ(tracker.ids.size(), 2U);
}

TEST(FeatureMasks, LoaderRejectsWrongChannelsAndDimensions)
{
    const std::string colorPath = temporaryPath("_color.png");
    const std::string sizePath = temporaryPath("_size.png");
    ASSERT_TRUE(cv::imwrite(colorPath, cv::Mat(10, 10, CV_8UC3, cv::Scalar(255, 255, 255))));
    ASSERT_TRUE(cv::imwrite(sizePath, cv::Mat(9, 10, CV_8UC1, cv::Scalar(255))));

    EXPECT_THROW(loadFeatureMask("/tmp/vins_mask_does_not_exist.png", 10, 10, "mask0"),
                 std::runtime_error);
    EXPECT_THROW(loadFeatureMask(colorPath, 10, 10, "mask0"), std::runtime_error);
    EXPECT_THROW(loadFeatureMask(sizePath, 10, 10, "mask0"), std::runtime_error);

    std::remove(colorPath.c_str());
    std::remove(sizePath.c_str());
}

} // namespace
