#include "ltv/ltv_types.h"
#include "ltv/ltv_velocity_oracle_gate.h"

#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <limits>
#include <string>

namespace
{

std::string temporaryPath(const char *name)
{
    return std::string("/tmp/") + name;
}

void writeMask(const std::string &path, const std::string &rows)
{
    std::ofstream stream(path);
    stream << "timestamp_ns,e_ltv,e_vins,advantage,oracle_0,oracle_002\n"
           << rows;
}

TEST(LtvVelocityOracleGate, DefaultsAreDisabled)
{
    const ltv::LtvConfig config;
    EXPECT_FALSE(config.enable_velocity_factor);
    EXPECT_FALSE(config.enable_gravity_factor);
    EXPECT_FALSE(config.enable_velocity_oracle_gate);
    EXPECT_TRUE(config.velocity_oracle_mask_path.empty());
    EXPECT_EQ(config.velocity_oracle_mask_column, "oracle_0");
}

TEST(LtvVelocityOracleGate, DisabledGatePassesThroughWithoutLoadingMask)
{
    ltv::VelocityOracleGate gate;
    ASSERT_TRUE(gate.configure(false, "", "oracle_0"));
    const ltv::VelocityOracleGateDecision decision = gate.evaluate(false, 42.0);
    EXPECT_FALSE(decision.loaded);
    EXPECT_FALSE(decision.hit);
    EXPECT_TRUE(decision.pass);
}

TEST(LtvVelocityOracleGate, UsesSelectedFrozenDecisionColumn)
{
    const std::string path = temporaryPath("ltv_velocity_oracle_mask.csv");
    writeMask(path,
        "1000000000,0.1,0.2,0.1,1,0\n"
        "1050000000,0.3,0.2,-0.1,0,0\n");

    ltv::VelocityOracleGate zero_margin_gate;
    ASSERT_TRUE(zero_margin_gate.configure(true, path, "oracle_0"));
    const auto zero_margin = zero_margin_gate.evaluate(true, 1.0);
    EXPECT_TRUE(zero_margin.loaded);
    EXPECT_TRUE(zero_margin.hit);
    EXPECT_TRUE(zero_margin.pass);

    ltv::VelocityOracleGate margin_gate;
    ASSERT_TRUE(margin_gate.configure(true, path, "oracle_002"));
    const auto margin = margin_gate.evaluate(true, 1.0);
    EXPECT_TRUE(margin.loaded);
    EXPECT_TRUE(margin.hit);
    EXPECT_FALSE(margin.pass);
    std::remove(path.c_str());
}

TEST(LtvVelocityOracleGate, RequiresExactTimestampAndBaseEligibility)
{
    const std::string path = temporaryPath("ltv_velocity_oracle_exact.csv");
    writeMask(path, "1000000000,0.1,0.2,0.1,1,1\n");
    ltv::VelocityOracleGate gate;
    ASSERT_TRUE(gate.configure(true, path, "oracle_0"));

    EXPECT_FALSE(gate.evaluate(false, 1.0).hit);
    const auto miss = gate.evaluate(true, 1.000000001);
    EXPECT_FALSE(miss.hit);
    EXPECT_FALSE(miss.pass);
    const auto nonfinite = gate.evaluate(
        true, std::numeric_limits<double>::quiet_NaN());
    EXPECT_FALSE(nonfinite.hit);
    EXPECT_FALSE(nonfinite.pass);
    std::remove(path.c_str());
}

TEST(LtvVelocityOracleGate, MissingMalformedAndInvalidColumnFailClosed)
{
    ltv::VelocityOracleGate missing;
    EXPECT_FALSE(missing.configure(
        true, "/tmp/no_such_velocity_oracle_mask.csv", "oracle_0"));
    EXPECT_FALSE(missing.evaluate(true, 1.0).pass);

    const std::string path = temporaryPath("ltv_velocity_oracle_malformed.csv");
    writeMask(path,
        "1000000000,0.1,0.2,0.1,1,0\n"
        "1000000000,0.1,0.2,0.1,1,0\n");
    ltv::VelocityOracleGate malformed;
    EXPECT_FALSE(malformed.configure(true, path, "oracle_0"));
    ltv::VelocityOracleGate invalid_column;
    EXPECT_FALSE(invalid_column.configure(true, path, "oracle_bad"));
    std::remove(path.c_str());
}

} // namespace
