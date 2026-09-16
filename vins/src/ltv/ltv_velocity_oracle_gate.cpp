#include "ltv_velocity_oracle_gate.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <vector>

namespace ltv
{
namespace
{

std::string trim(const std::string &value)
{
    const std::string whitespace = " \t\r\n";
    const std::size_t first = value.find_first_not_of(whitespace);
    if (first == std::string::npos)
        return std::string();
    return value.substr(first, value.find_last_not_of(whitespace) - first + 1);
}

std::vector<std::string> splitCsv(const std::string &line)
{
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ','))
        fields.push_back(trim(field));
    return fields;
}

std::string columnName(const std::string &value)
{
    std::string name = trim(value);
    if (!name.empty() && name.front() == '#')
        name.erase(name.begin());
    const std::size_t units = name.find(" [");
    if (units != std::string::npos)
        name.erase(units);
    return trim(name);
}

bool parseTimestamp(const std::string &value, std::int64_t &result)
{
    try
    {
        std::size_t parsed = 0;
        result = std::stoll(value, &parsed);
        return parsed == value.size() && result >= 0;
    }
    catch (const std::exception &)
    {
        return false;
    }
}

} // namespace

bool VelocityOracleGate::configure(bool enabled, const std::string &mask_path,
                                   const std::string &mask_column)
{
    enabled_ = enabled;
    loaded_ = false;
    mask_.clear();

    if (!enabled_)
        return true;
    if (mask_path.empty() ||
        (mask_column != "oracle_0" && mask_column != "oracle_002"))
        return false;

    std::ifstream stream(mask_path);
    if (!stream)
        return false;

    std::string line;
    if (!std::getline(stream, line))
        return false;
    const std::vector<std::string> header = splitCsv(line);
    std::unordered_map<std::string, std::size_t> columns;
    for (std::size_t index = 0; index < header.size(); ++index)
        columns[columnName(header[index])] = index;

    const std::vector<std::string> required = {"timestamp_ns", mask_column};
    for (const std::string &name : required)
        if (columns.count(name) == 0)
            return false;

    std::int64_t previous_timestamp = -1;
    while (std::getline(stream, line))
    {
        if (trim(line).empty())
            continue;
        const std::vector<std::string> fields = splitCsv(line);
        const std::size_t timestamp_index = columns.at("timestamp_ns");
        const std::size_t decision_index = columns.at(mask_column);
        std::int64_t timestamp_ns = 0;
        if (timestamp_index >= fields.size() || decision_index >= fields.size() ||
            !parseTimestamp(fields[timestamp_index], timestamp_ns) ||
            (fields[decision_index] != "0" && fields[decision_index] != "1") ||
            (!mask_.empty() && timestamp_ns <= previous_timestamp))
        {
            mask_.clear();
            return false;
        }
        previous_timestamp = timestamp_ns;
        mask_.emplace(timestamp_ns, fields[decision_index] == "1");
    }

    loaded_ = !mask_.empty();
    return loaded_;
}

VelocityOracleGateDecision VelocityOracleGate::evaluate(
    bool base_eligible, double frame_timestamp) const
{
    VelocityOracleGateDecision decision;
    if (!enabled_)
    {
        decision.pass = true;
        return decision;
    }

    decision.loaded = loaded_;
    if (!loaded_ || !base_eligible || !std::isfinite(frame_timestamp) ||
        frame_timestamp < 0.0 ||
        frame_timestamp > static_cast<double>(
            std::numeric_limits<std::int64_t>::max()) * 1e-9)
        return decision;

    const std::int64_t timestamp_ns = static_cast<std::int64_t>(
        std::llround(frame_timestamp * 1e9));
    const auto entry = mask_.find(timestamp_ns);
    if (entry == mask_.end())
        return decision;
    decision.hit = true;
    decision.pass = entry->second;
    return decision;
}

} // namespace ltv
