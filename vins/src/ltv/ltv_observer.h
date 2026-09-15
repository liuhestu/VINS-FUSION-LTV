#pragma once

#include "ltv_types.h"

#include <unordered_map>
#include <vector>

namespace ltv
{

class LtvObserver
{
  public:
    LtvObserver();

    void configure(const LtvConfig &config);
    void reset(LtvResetReason reason = LtvResetReason::EstimatorReset);
    void start(double imu_timestamp);

    void propagateImu(double dt,
                      const Eigen::Vector3d &acc_measurement,
                      const Eigen::Vector3d &gyro_measurement,
                      const Eigen::Vector3d &accel_bias,
                      const Eigen::Vector3d &gyro_bias);

    LtvSnapshot updateFeatures(double frame_timestamp,
                               double imu_timestamp,
                               const std::vector<LtvFeatureObservation> &observations,
                               const Eigen::Matrix3d &rotation_body_camera,
                               const Eigen::Vector3d &position_body_camera);

    LtvSnapshot snapshot(double frame_timestamp = 0.0) const;
    bool enabled() const;
    bool started() const;

    // Read-only accessors are intentionally provided for formula and lifecycle tests.
    const Eigen::VectorXd &state() const;
    const Eigen::MatrixXd &covariance() const;
    int slotForFeature(int feature_id) const;

  private:
    void initializeBaseState();
    void updateFeatureLifecycle(const std::vector<LtvFeatureObservation> &observations);
    void rebuildState(const std::vector<int> &feature_ids);
    bool sanitizeCovariance();
    bool stateFinite() const;
    Eigen::MatrixXd processNoise() const;
    int velocityOffset() const;
    int gravityOffset() const;

    LtvConfig config_;
    bool configured_ = false;
    bool started_ = false;
    double imu_timestamp_ = 0.0;
    double last_camera_imu_timestamp_ = -1.0;
    double last_frame_timestamp_ = -1.0;
    int healthy_camera_updates_ = 0;
    int observed_features_ = 0;
    double innovation_norm_ = 0.0;
    double update_time_ms_ = 0.0;
    int camera_substeps_ = 0;
    LtvResetReason last_reset_reason_ = LtvResetReason::None;

    Eigen::VectorXd state_;
    Eigen::MatrixXd covariance_;
    std::unordered_map<int, int> feature_to_slot_;
    std::unordered_map<int, int> missed_frames_;
    std::unordered_map<int, int> candidate_age_;
};

} // namespace ltv
