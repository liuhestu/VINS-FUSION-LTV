#pragma once

#include <eigen3/Eigen/Dense>

#include <string>

namespace ltv
{

struct LtvConfig
{
    bool enable = false;
    bool log_debug = false;
    int max_features = 30;
    int min_features = 15;
    int max_missed_frames = 2;
    int warmup_camera_updates = 20;
    double max_imu_dt = 0.05;
    double reset_gap = 0.2;
    double timestamp_tolerance = 0.005;
    double q_landmark = 1e-4;
    double v_landmark = 1e6;
    double v_velocity = 1e6;
    double v_gravity = 1e6;
    double initial_p_landmark = 1.0;
    double initial_p_velocity = 1.0;
    double initial_p_gravity = 1.0;
    double covariance_floor = 1e-9;
    double covariance_failure_threshold = -1e-3;
    double camera_euler_safety = 0.5;
    int max_camera_substeps = 100;
    double gravity_norm_min = 7.0;
    double gravity_norm_max = 12.0;
    std::string debug_csv_path;
};

struct LtvFeatureObservation
{
    int feature_id = -1;
    Eigen::Vector3d normalized_coordinate = Eigen::Vector3d::Zero();
};

enum class LtvResetReason
{
    None,
    EstimatorReset,
    Reconfigured,
    TimestampBackward,
    TimestampGap,
    InvalidInput,
    InvalidState,
    CovarianceFailure
};

inline const char *toString(LtvResetReason reason)
{
    switch (reason)
    {
        case LtvResetReason::None: return "none";
        case LtvResetReason::EstimatorReset: return "estimator_reset";
        case LtvResetReason::Reconfigured: return "reconfigured";
        case LtvResetReason::TimestampBackward: return "timestamp_backward";
        case LtvResetReason::TimestampGap: return "timestamp_gap";
        case LtvResetReason::InvalidInput: return "invalid_input";
        case LtvResetReason::InvalidState: return "invalid_state";
        case LtvResetReason::CovarianceFailure: return "covariance_failure";
    }
    return "unknown";
}

struct LtvSnapshot
{
    double frame_timestamp = 0.0;
    double imu_timestamp = 0.0;
    Eigen::Vector3d velocity_body = Eigen::Vector3d::Zero();
    Eigen::Vector3d gravity_body = Eigen::Vector3d::Zero();
    bool valid = false;
    bool velocity_valid = false;
    bool gravity_valid = false;
    int state_features = 0;
    int observed_features = 0;
    int healthy_camera_updates = 0;
    double innovation_norm = 0.0;
    double covariance_trace = 0.0;
    double covariance_min_diagonal = 0.0;
    double covariance_max_diagonal = 0.0;
    double update_time_ms = 0.0;
    int camera_substeps = 0;
    LtvResetReason last_reset_reason = LtvResetReason::None;
};

} // namespace ltv
