// Offline-only deterministic feeder; it never subscribes to ROS/DDS.
#include <rclcpp/rclcpp.hpp>

#include <opencv2/imgcodecs.hpp>

#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/stereo_synchronizer.h"
#include "utility/visualization.h"

namespace
{

struct Imu
{
    std::int64_t timestamp_ns;
    Eigen::Vector3d acceleration;
    Eigen::Vector3d angular_velocity;
};

struct Frame
{
    std::size_t pair_index;
    std::int64_t timestamp_ns;
    std::int64_t left_timestamp_ns;
    std::int64_t right_timestamp_ns;
    std::string left_path;
    std::string right_path;
};

std::vector<std::string> fields(const std::string &line)
{
    std::vector<std::string> result;
    std::stringstream stream(line);
    std::string value;
    while (std::getline(stream, value, ','))
    {
        if (!value.empty() && value.back() == '\r')
            value.pop_back();
        result.push_back(value);
    }
    return result;
}

std::vector<Imu> readImu(const std::string &path)
{
    std::ifstream stream(path);
    std::string line;
    if (!std::getline(stream, line) ||
        line != "timestamp_ns,ax,ay,az,gx,gy,gz")
    {
        throw std::runtime_error("invalid IMU manifest header");
    }
    std::vector<Imu> result;
    std::int64_t previous = -1;
    while (std::getline(stream, line))
    {
        const auto values = fields(line);
        if (values.size() != 7)
            throw std::runtime_error("invalid IMU manifest row");
        Imu event{
            std::stoll(values[0]),
            Eigen::Vector3d(
                std::stod(values[1]), std::stod(values[2]), std::stod(values[3])),
            Eigen::Vector3d(
                std::stod(values[4]), std::stod(values[5]), std::stod(values[6]))};
        if (event.timestamp_ns <= previous || !event.acceleration.allFinite() ||
            !event.angular_velocity.allFinite())
        {
            throw std::runtime_error(
                "IMU manifest is not strictly increasing and finite");
        }
        previous = event.timestamp_ns;
        result.push_back(event);
    }
    return result;
}

std::vector<Frame> readFrames(const std::string &path)
{
    std::ifstream stream(path);
    std::string line;
    if (!std::getline(stream, line) ||
        line != "pair_index,pair_timestamp,left_timestamp,right_timestamp,"
                "left_index,right_index,left_png,right_png")
    {
        throw std::runtime_error("invalid canonical stereo manifest header");
    }
    std::vector<Frame> result;
    std::int64_t previous = -1;
    while (std::getline(stream, line))
    {
        const auto values = fields(line);
        if (values.size() != 8)
            throw std::runtime_error("invalid canonical stereo manifest row");
        const Frame frame{
            static_cast<std::size_t>(std::stoull(values[0])),
            std::stoll(values[1]), std::stoll(values[2]), std::stoll(values[3]),
            values[6], values[7]};
        if (frame.pair_index != result.size() ||
            frame.timestamp_ns != frame.left_timestamp_ns ||
            frame.timestamp_ns <= previous ||
            vins::classifyStereoTimestamps(
                frame.left_timestamp_ns, frame.right_timestamp_ns) !=
                vins::StereoSyncDecision::Match)
        {
            throw std::runtime_error(
                "canonical stereo manifest is invalid or not strictly increasing");
        }
        previous = frame.timestamp_ns;
        result.push_back(frame);
    }
    return result;
}

void writeReplaySummary(std::size_t canonical_count, std::size_t consumed_count)
{
    std::ofstream stream(OUTPUT_FOLDER + "/replay_summary.json",
                         std::ios::out | std::ios::trunc);
    if (!stream)
        throw std::runtime_error("cannot write replay summary");
    stream << "{\n"
           << "  \"canonical_pair_count\": " << canonical_count << ",\n"
           << "  \"consumed_pair_count\": " << consumed_count << "\n"
           << "}\n";
}

int runReplay(const std::string &config, const std::string &cache)
{
    const auto imu = readImu(cache + "/imu.csv");
    const auto frames = readFrames(cache + "/canonical_stereo_pairs.csv");
    if (imu.empty() || frames.empty())
        throw std::runtime_error("empty Stage 5 cache");

    auto node = rclcpp::Node::make_shared("stage5_replay");
    try
    {
        readParameters(config);
        MULTIPLE_THREAD = 0;
        registerPub(node);
        std::size_t consumed_count = 0;
        {
            Estimator estimator;
            estimator.setParameter();
            std::size_t next_imu = 0;
            for (const auto &frame : frames)
            {
                while (next_imu < imu.size() &&
                       imu[next_imu].timestamp_ns < frame.timestamp_ns)
                {
                    const auto &event = imu[next_imu++];
                    estimator.inputIMU(
                        event.timestamp_ns * 1e-9, event.acceleration,
                        event.angular_velocity);
                }
                if (next_imu == imu.size())
                    throw std::runtime_error(
                        "cache ends before an IMU sample for a frame");
                const auto &event = imu[next_imu++];
                estimator.inputIMU(
                    event.timestamp_ns * 1e-9, event.acceleration,
                    event.angular_velocity);
                const cv::Mat left = cv::imread(
                    cache + "/" + frame.left_path, cv::IMREAD_GRAYSCALE);
                const cv::Mat right = cv::imread(
                    cache + "/" + frame.right_path, cv::IMREAD_GRAYSCALE);
                if (left.empty() || right.empty())
                    throw std::runtime_error("cache image is missing or unreadable");
                estimator.inputImage(frame.timestamp_ns * 1e-9, left, right);
                ++consumed_count;
            }
            while (next_imu < imu.size())
            {
                const auto &event = imu[next_imu++];
                estimator.inputIMU(
                    event.timestamp_ns * 1e-9, event.acceleration,
                    event.angular_velocity);
            }
            if (!estimator.finishInputAndDrain())
                throw std::runtime_error(
                    "estimator did not drain all queued features");
        } // Estimator and its CSV streams are destroyed before ROS entities.
        writeReplaySummary(frames.size(), consumed_count);
        unregisterPub();
        node.reset();
        return 0;
    }
    catch (...)
    {
        unregisterPub();
        node.reset();
        throw;
    }
}

} // namespace

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        std::cerr << "usage: stage5_replay CONFIG_YAML CACHE_DIRECTORY\n";
        return 2;
    }
    rclcpp::init(argc, argv);
    try
    {
        const int result = runReplay(argv[1], argv[2]);
        rclcpp::shutdown();
        return result;
    }
    catch (const std::exception &error)
    {
        std::cerr << "stage5_replay: " << error.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
}
