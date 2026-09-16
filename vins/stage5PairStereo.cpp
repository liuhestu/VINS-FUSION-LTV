#include "utility/stereo_synchronizer.h"

#include <cstdint>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

struct TimestampEntry
{
    std::int64_t timestamp_ns;
    std::size_t source_index;
};

std::vector<TimestampEntry> readTimestamps(const std::string &path)
{
    std::ifstream stream(path);
    if (!stream)
        throw std::runtime_error("cannot open timestamp input: " + path);

    std::string line;
    if (!std::getline(stream, line) || line != "timestamp_ns,source_index")
        throw std::runtime_error("invalid timestamp input header: " + path);

    std::vector<TimestampEntry> entries;
    std::int64_t previous = -1;
    while (std::getline(stream, line))
    {
        if (line.empty())
            continue;
        std::stringstream fields(line);
        std::string timestamp;
        std::string index;
        if (!std::getline(fields, timestamp, ',') ||
            !std::getline(fields, index, ',') || fields.rdbuf()->in_avail() != 0)
        {
            throw std::runtime_error("invalid timestamp input row: " + path);
        }
        const std::int64_t value = std::stoll(timestamp);
        if (value < 0 || (!entries.empty() && value <= previous))
            throw std::runtime_error("timestamps are not strictly increasing: " + path);
        previous = value;
        entries.push_back({value, static_cast<std::size_t>(std::stoull(index))});
    }
    return entries;
}

void writePairs(const std::string &path, const std::vector<TimestampEntry> &left,
                const std::vector<TimestampEntry> &right)
{
    std::ofstream stream(path, std::ios::out | std::ios::trunc);
    if (!stream)
        throw std::runtime_error("cannot open pair output: " + path);
    stream << "pair_index,pair_timestamp,left_timestamp,right_timestamp,"
              "left_index,right_index\n";

    std::size_t left_index = 0;
    std::size_t right_index = 0;
    std::size_t pair_index = 0;
    while (left_index < left.size() && right_index < right.size())
    {
        switch (vins::classifyStereoTimestamps(
            left[left_index].timestamp_ns, right[right_index].timestamp_ns))
        {
            case vins::StereoSyncDecision::DropLeft:
                ++left_index;
                break;
            case vins::StereoSyncDecision::DropRight:
                ++right_index;
                break;
            case vins::StereoSyncDecision::Match:
                stream << pair_index++ << ',' << left[left_index].timestamp_ns << ','
                       << left[left_index].timestamp_ns << ','
                       << right[right_index].timestamp_ns << ','
                       << left[left_index].source_index << ','
                       << right[right_index].source_index << '\n';
                ++left_index;
                ++right_index;
                break;
        }
    }
}

} // namespace

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        std::cerr << "usage: stage5_pair_stereo LEFT.csv RIGHT.csv OUTPUT.csv\n";
        return 2;
    }
    try
    {
        writePairs(argv[3], readTimestamps(argv[1]), readTimestamps(argv[2]));
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "stage5_pair_stereo: " << error.what() << '\n';
        return 1;
    }
}
