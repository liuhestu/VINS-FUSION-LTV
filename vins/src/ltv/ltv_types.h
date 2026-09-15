#pragma once

#include <eigen3/Eigen/Dense>

#include <string>

namespace ltv
{

struct LtvConfig
{
    bool enable = false;
    bool log_debug = false;
    bool enable_gravity_factor = false;
    bool enable_velocity_factor = false;
    bool enable_gravity_quality_gate = false;
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
    double gravity_sigma_deg = 10.0;
    double gravity_huber_delta = 2.0;
    int gravity_gate_min_features = 15;
    double gravity_gate_max_eta_norm_error = 0.5;
    double gravity_gate_max_normalized_innovation = -1.0;
    int gravity_gate_reset_cooldown_frames = 0;
    double velocity_sigma_mps = 1.0;
    double velocity_huber_delta = 2.0;
    double snapshot_max_time_error = 0.005;
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
    double snapshot_time_error = 0.0;
    double gravity_angle_ltv_vs_vins = 0.0;
    double gravity_factor_residual_norm = 0.0;
    double gravity_factor_weighted_residual_norm = 0.0;
    bool gravity_factor_added = false;
    bool gravity_gate_base_eligible = false;
    bool gravity_gate_feature_ok = false;
    bool gravity_gate_eta_norm_ok = false;
    bool gravity_gate_innovation_ok = false;
    bool gravity_gate_reset_ok = false;
    bool gravity_gate_pass = false;
    unsigned int gravity_gate_reason_mask = 0;
    Eigen::Vector3d vins_velocity_body = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity_factor_residual = Eigen::Vector3d::Zero();
    double velocity_factor_residual_norm = 0.0;
    double velocity_factor_weighted_residual_norm = 0.0;
    bool velocity_factor_added = false;
};

} // namespace ltv
