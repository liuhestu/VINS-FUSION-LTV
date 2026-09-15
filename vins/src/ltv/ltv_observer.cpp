#include "ltv_observer.h"

#include "../utility/utility.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <set>

namespace ltv
{
namespace
{

bool finiteVector(const Eigen::Vector3d &value)
{
    return value.allFinite();
}

} // namespace

LtvObserver::LtvObserver()
{
    initializeBaseState();
}

void LtvObserver::configure(const LtvConfig &config)
{
    config_ = config;
    config_.max_features = std::max(1, config_.max_features);
    config_.min_features = std::max(1, std::min(config_.min_features, config_.max_features));
    config_.max_missed_frames = std::max(0, config_.max_missed_frames);
    config_.warmup_camera_updates = std::max(1, config_.warmup_camera_updates);
    config_.camera_euler_safety = std::max(1e-3, config_.camera_euler_safety);
    config_.max_camera_substeps = std::max(1, config_.max_camera_substeps);
    configured_ = true;
    reset(LtvResetReason::Reconfigured);
}

void LtvObserver::reset(LtvResetReason reason)
{
    started_ = false;
    imu_timestamp_ = 0.0;
    last_camera_imu_timestamp_ = -1.0;
    last_frame_timestamp_ = -1.0;
    healthy_camera_updates_ = 0;
    observed_features_ = 0;
    innovation_norm_ = 0.0;
    update_time_ms_ = 0.0;
    camera_substeps_ = 0;
    last_reset_reason_ = reason;
    feature_to_slot_.clear();
    missed_frames_.clear();
    candidate_age_.clear();
    initializeBaseState();
}

void LtvObserver::initializeBaseState()
{
    state_ = Eigen::VectorXd::Zero(6);
    covariance_ = Eigen::MatrixXd::Zero(6, 6);
    covariance_.topLeftCorner<3, 3>() =
        config_.initial_p_velocity * Eigen::Matrix3d::Identity();
    covariance_.bottomRightCorner<3, 3>() =
        config_.initial_p_gravity * Eigen::Matrix3d::Identity();
}

void LtvObserver::start(double imu_timestamp)
{
    if (!configured_ || !config_.enable || !std::isfinite(imu_timestamp))
        return;

    reset(LtvResetReason::None);
    started_ = true;
    imu_timestamp_ = imu_timestamp;
}

bool LtvObserver::enabled() const
{
    return configured_ && config_.enable;
}

bool LtvObserver::started() const
{
    return started_;
}

int LtvObserver::velocityOffset() const
{
    return 3 * static_cast<int>(feature_to_slot_.size());
}

int LtvObserver::gravityOffset() const
{
    return velocityOffset() + 3;
}

Eigen::MatrixXd LtvObserver::processNoise() const
{
    Eigen::MatrixXd noise = Eigen::MatrixXd::Zero(state_.size(), state_.size());
    const int landmark_dimension = velocityOffset();
    if (landmark_dimension > 0)
    {
        noise.topLeftCorner(landmark_dimension, landmark_dimension).diagonal().setConstant(
            config_.v_landmark);
    }
    noise.block<3, 3>(velocityOffset(), velocityOffset()).diagonal().setConstant(config_.v_velocity);
    noise.block<3, 3>(gravityOffset(), gravityOffset()).diagonal().setConstant(config_.v_gravity);
    return noise;
}

void LtvObserver::propagateImu(double dt,
                               const Eigen::Vector3d &acc_measurement,
                               const Eigen::Vector3d &gyro_measurement,
                               const Eigen::Vector3d &accel_bias,
                               const Eigen::Vector3d &gyro_bias)
{
    if (!enabled() || !started_)
        return;

    if (!std::isfinite(dt) || dt <= 0.0 || dt > config_.max_imu_dt ||
        !finiteVector(acc_measurement) || !finiteVector(gyro_measurement) ||
        !finiteVector(accel_bias) || !finiteVector(gyro_bias))
    {
        reset(dt > config_.max_imu_dt ? LtvResetReason::TimestampGap : LtvResetReason::InvalidInput);
        return;
    }

    const Eigen::Vector3d omega = gyro_measurement - gyro_bias;
    const Eigen::Vector3d acceleration = acc_measurement - accel_bias;
    const Eigen::Matrix3d negative_omega_cross = -Utility::skewSymmetric(omega);
    const int landmark_count = static_cast<int>(feature_to_slot_.size());
    const int dimension = static_cast<int>(state_.size());

    Eigen::MatrixXd system = Eigen::MatrixXd::Zero(dimension, dimension);
    Eigen::MatrixXd input = Eigen::MatrixXd::Zero(dimension, 3);
    for (int slot = 0; slot < landmark_count; ++slot)
    {
        system.block<3, 3>(3 * slot, 3 * slot) = negative_omega_cross;
        system.block<3, 3>(3 * slot, velocityOffset()) = -Eigen::Matrix3d::Identity();
    }
    system.block<3, 3>(velocityOffset(), velocityOffset()) = negative_omega_cross;
    system.block<3, 3>(velocityOffset(), gravityOffset()) = Eigen::Matrix3d::Identity();
    system.block<3, 3>(gravityOffset(), gravityOffset()) = negative_omega_cross;
    input.block<3, 3>(velocityOffset(), 0) = Eigen::Matrix3d::Identity();

    state_ += dt * (system * state_ + input * acceleration);
    covariance_ += dt * (system * covariance_ + covariance_ * system.transpose() + processNoise());
    imu_timestamp_ += dt;

    if (!stateFinite())
    {
        reset(LtvResetReason::InvalidState);
        return;
    }
    if (!sanitizeCovariance())
        reset(LtvResetReason::CovarianceFailure);
}

void LtvObserver::updateFeatureLifecycle(const std::vector<LtvFeatureObservation> &observations)
{
    std::set<int> observed_ids;
    for (const auto &observation : observations)
    {
        if (observation.feature_id >= 0 && observation.normalized_coordinate.allFinite() &&
            observation.normalized_coordinate.norm() > 1e-12)
        {
            observed_ids.insert(observation.feature_id);
        }
    }

    for (auto candidate = candidate_age_.begin(); candidate != candidate_age_.end();)
    {
        if (observed_ids.count(candidate->first) == 0 && feature_to_slot_.count(candidate->first) == 0)
            candidate = candidate_age_.erase(candidate);
        else
            ++candidate;
    }
    for (int feature_id : observed_ids)
        candidate_age_[feature_id] += 1;

    std::vector<int> active_ids(feature_to_slot_.size(), -1);
    for (const auto &entry : feature_to_slot_)
        active_ids[entry.second] = entry.first;

    std::vector<int> retained_ids;
    retained_ids.reserve(active_ids.size());
    for (int feature_id : active_ids)
    {
        if (observed_ids.count(feature_id) != 0)
            missed_frames_[feature_id] = 0;
        else
            missed_frames_[feature_id] += 1;

        if (missed_frames_[feature_id] <= config_.max_missed_frames)
            retained_ids.push_back(feature_id);
        else
        {
            missed_frames_.erase(feature_id);
            candidate_age_.erase(feature_id);
        }
    }

    std::vector<int> candidates;
    for (int feature_id : observed_ids)
    {
        if (feature_to_slot_.count(feature_id) == 0)
            candidates.push_back(feature_id);
    }
    std::sort(candidates.begin(), candidates.end(), [this](int lhs, int rhs) {
        const int lhs_age = candidate_age_.at(lhs);
        const int rhs_age = candidate_age_.at(rhs);
        return lhs_age != rhs_age ? lhs_age > rhs_age : lhs < rhs;
    });

    for (int feature_id : candidates)
    {
        if (static_cast<int>(retained_ids.size()) >= config_.max_features)
            break;
        retained_ids.push_back(feature_id);
        missed_frames_[feature_id] = 0;
    }

    if (retained_ids != active_ids)
        rebuildState(retained_ids);
}

void LtvObserver::rebuildState(const std::vector<int> &feature_ids)
{
    const int old_feature_count = static_cast<int>(feature_to_slot_.size());
    const int old_velocity_offset = 3 * old_feature_count;
    const int old_gravity_offset = old_velocity_offset + 3;
    const int new_velocity_offset = 3 * static_cast<int>(feature_ids.size());
    const int new_gravity_offset = new_velocity_offset + 3;
    const int new_dimension = new_gravity_offset + 3;

    Eigen::VectorXd new_state = Eigen::VectorXd::Zero(new_dimension);
    Eigen::MatrixXd new_covariance = Eigen::MatrixXd::Zero(new_dimension, new_dimension);
    std::vector<int> old_index(new_dimension, -1);
    std::unordered_map<int, int> new_feature_to_slot;

    for (std::size_t slot = 0; slot < feature_ids.size(); ++slot)
    {
        const int feature_id = feature_ids[slot];
        new_feature_to_slot[feature_id] = static_cast<int>(slot);
        const auto old_slot = feature_to_slot_.find(feature_id);
        if (old_slot != feature_to_slot_.end())
        {
            for (int axis = 0; axis < 3; ++axis)
                old_index[3 * static_cast<int>(slot) + axis] = 3 * old_slot->second + axis;
        }
    }
    for (int axis = 0; axis < 3; ++axis)
    {
        old_index[new_velocity_offset + axis] = old_velocity_offset + axis;
        old_index[new_gravity_offset + axis] = old_gravity_offset + axis;
    }

    for (int row = 0; row < new_dimension; ++row)
    {
        if (old_index[row] >= 0)
            new_state[row] = state_[old_index[row]];
        for (int column = 0; column < new_dimension; ++column)
        {
            if (old_index[row] >= 0 && old_index[column] >= 0)
                new_covariance(row, column) = covariance_(old_index[row], old_index[column]);
        }
    }

    for (std::size_t slot = 0; slot < feature_ids.size(); ++slot)
    {
        if (feature_to_slot_.count(feature_ids[slot]) == 0)
        {
            new_covariance.block<3, 3>(3 * static_cast<int>(slot), 3 * static_cast<int>(slot)) =
                config_.initial_p_landmark * Eigen::Matrix3d::Identity();
        }
    }

    state_.swap(new_state);
    covariance_.swap(new_covariance);
    feature_to_slot_.swap(new_feature_to_slot);
}

LtvSnapshot LtvObserver::updateFeatures(
    double frame_timestamp,
    double imu_timestamp,
    const std::vector<LtvFeatureObservation> &observations,
    const Eigen::Matrix3d &rotation_body_camera,
    const Eigen::Vector3d &position_body_camera)
{
    const auto update_start = std::chrono::steady_clock::now();
    if (!enabled() || !started_)
        return snapshot(frame_timestamp);

    if (!std::isfinite(frame_timestamp) || !std::isfinite(imu_timestamp) ||
        !rotation_body_camera.allFinite() || !position_body_camera.allFinite())
    {
        reset(LtvResetReason::InvalidInput);
        return snapshot(frame_timestamp);
    }
    if (!(rotation_body_camera * rotation_body_camera.transpose())
             .isApprox(Eigen::Matrix3d::Identity(), 1e-6) ||
        rotation_body_camera.determinant() <= 0.0)
    {
        reset(LtvResetReason::InvalidInput);
        return snapshot(frame_timestamp);
    }
    if ((last_frame_timestamp_ >= 0.0 && frame_timestamp <= last_frame_timestamp_) ||
        imu_timestamp + config_.timestamp_tolerance < imu_timestamp_)
    {
        reset(LtvResetReason::TimestampBackward);
        return snapshot(frame_timestamp);
    }
    if (std::abs(imu_timestamp - imu_timestamp_) > config_.timestamp_tolerance)
    {
        reset(LtvResetReason::TimestampGap);
        return snapshot(frame_timestamp);
    }
    if (last_camera_imu_timestamp_ >= 0.0 &&
        imu_timestamp - last_camera_imu_timestamp_ > config_.reset_gap)
    {
        reset(LtvResetReason::TimestampGap);
        return snapshot(frame_timestamp);
    }

    updateFeatureLifecycle(observations);

    std::unordered_map<int, Eigen::Vector3d> coordinate_by_id;
    for (const auto &observation : observations)
    {
        if (feature_to_slot_.count(observation.feature_id) != 0 &&
            observation.normalized_coordinate.allFinite() &&
            observation.normalized_coordinate.norm() > 1e-12)
        {
            coordinate_by_id.emplace(observation.feature_id, observation.normalized_coordinate);
        }
    }

    std::vector<std::pair<int, int>> observed_slots;
    observed_slots.reserve(coordinate_by_id.size());
    for (const auto &entry : coordinate_by_id)
        observed_slots.emplace_back(feature_to_slot_.at(entry.first), entry.first);
    std::sort(observed_slots.begin(), observed_slots.end());
    observed_features_ = static_cast<int>(observed_slots.size());

    Eigen::MatrixXd measurement = Eigen::MatrixXd::Zero(3 * observed_features_, state_.size());
    Eigen::VectorXd output = Eigen::VectorXd::Zero(3 * observed_features_);
    for (int index = 0; index < observed_features_; ++index)
    {
        const int slot = observed_slots[index].first;
        const int feature_id = observed_slots[index].second;
        const Eigen::Vector3d bearing_camera = coordinate_by_id.at(feature_id).normalized();
        const Eigen::Vector3d bearing_body = rotation_body_camera * bearing_camera;
        const Eigen::Matrix3d projection =
            Eigen::Matrix3d::Identity() - bearing_body * bearing_body.transpose();
        measurement.block<3, 3>(3 * index, 3 * slot) = projection;
        output.segment<3>(3 * index) = projection * position_body_camera;
    }

    const Eigen::VectorXd innovation = output - measurement * state_;
    innovation_norm_ = innovation.norm();
    if (last_camera_imu_timestamp_ >= 0.0 && observed_features_ > 0)
    {
        const double camera_dt = imu_timestamp - last_camera_imu_timestamp_;
        const Eigen::MatrixXd projected_covariance =
            config_.q_landmark * measurement * covariance_ * measurement.transpose();
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> projected_solver(projected_covariance);
        if (projected_solver.info() != Eigen::Success ||
            !projected_solver.eigenvalues().allFinite())
        {
            reset(LtvResetReason::CovarianceFailure);
            return snapshot(frame_timestamp);
        }
        const double maximum_rate = std::max(0.0, projected_solver.eigenvalues().maxCoeff());
        camera_substeps_ = std::max(
            1, static_cast<int>(std::ceil(camera_dt * maximum_rate /
                                          config_.camera_euler_safety)));
        if (camera_substeps_ > config_.max_camera_substeps)
        {
            reset(LtvResetReason::CovarianceFailure);
            return snapshot(frame_timestamp);
        }
        const double correction_dt = camera_dt / camera_substeps_;
        // Engineering Euler discretization of the paper's continuous-time
        // Riccati observer. Adaptive substeps keep the explicit update inside
        // its positive-semidefinite stability region. This is not a discrete
        // equation stated by the paper; C and y remain frozen for this frame.
        for (int step = 0; step < camera_substeps_; ++step)
        {
            const Eigen::VectorXd substep_innovation = output - measurement * state_;
            const Eigen::MatrixXd covariance_measurement_transpose =
                covariance_ * measurement.transpose();
            state_ += correction_dt * config_.q_landmark *
                      covariance_measurement_transpose * substep_innovation;
            covariance_ -= correction_dt * config_.q_landmark *
                           covariance_measurement_transpose * measurement * covariance_;
            if (!sanitizeCovariance())
            {
                reset(LtvResetReason::CovarianceFailure);
                return snapshot(frame_timestamp);
            }
        }
    }
    else
        camera_substeps_ = 0;

    last_camera_imu_timestamp_ = imu_timestamp;
    last_frame_timestamp_ = frame_timestamp;
    if (!stateFinite())
    {
        reset(LtvResetReason::InvalidState);
        return snapshot(frame_timestamp);
    }
    if (!sanitizeCovariance())
    {
        reset(LtvResetReason::CovarianceFailure);
        return snapshot(frame_timestamp);
    }

    if (observed_features_ >= config_.min_features)
        ++healthy_camera_updates_;
    else
        healthy_camera_updates_ = 0;

    update_time_ms_ = std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - update_start)
                          .count();
    return snapshot(frame_timestamp);
}

bool LtvObserver::stateFinite() const
{
    return state_.allFinite() && covariance_.allFinite();
}

bool LtvObserver::sanitizeCovariance()
{
    covariance_ = 0.5 * (covariance_ + covariance_.transpose());
    if (!covariance_.allFinite())
        return false;

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver(covariance_);
    if (solver.info() != Eigen::Success || !solver.eigenvalues().allFinite())
        return false;
    const double spectral_scale = std::max(1.0, solver.eigenvalues().cwiseAbs().maxCoeff());
    // Explicit Euler integration may create a small negative eigenvalue at the
    // camera rate. Treat the configured negative threshold as a relative
    // spectral tolerance; larger violations indicate a genuinely broken P.
    if (solver.eigenvalues().minCoeff() <
        config_.covariance_failure_threshold * spectral_scale)
        return false;

    Eigen::VectorXd eigenvalues = solver.eigenvalues().cwiseMax(config_.covariance_floor);
    covariance_ = solver.eigenvectors() * eigenvalues.asDiagonal() * solver.eigenvectors().transpose();
    covariance_ = 0.5 * (covariance_ + covariance_.transpose());
    return covariance_.allFinite();
}

LtvSnapshot LtvObserver::snapshot(double frame_timestamp) const
{
    LtvSnapshot result;
    result.frame_timestamp = frame_timestamp != 0.0 ? frame_timestamp : last_frame_timestamp_;
    result.imu_timestamp = imu_timestamp_;
    result.state_features = static_cast<int>(feature_to_slot_.size());
    result.observed_features = observed_features_;
    result.healthy_camera_updates = healthy_camera_updates_;
    result.innovation_norm = innovation_norm_;
    result.update_time_ms = update_time_ms_;
    result.camera_substeps = camera_substeps_;
    result.last_reset_reason = last_reset_reason_;

    if (state_.size() >= 6)
    {
        result.velocity_body = state_.segment<3>(velocityOffset());
        result.gravity_body = state_.segment<3>(gravityOffset());
    }
    if (covariance_.size() > 0 && covariance_.allFinite())
    {
        result.covariance_trace = covariance_.trace();
        result.covariance_min_diagonal = covariance_.diagonal().minCoeff();
        result.covariance_max_diagonal = covariance_.diagonal().maxCoeff();
    }

    result.valid = enabled() && started_ && stateFinite() &&
                   healthy_camera_updates_ >= config_.warmup_camera_updates;
    result.velocity_valid = result.valid && result.velocity_body.allFinite();
    const double gravity_norm = result.gravity_body.norm();
    result.gravity_valid = result.valid && result.gravity_body.allFinite() &&
                           gravity_norm >= config_.gravity_norm_min &&
                           gravity_norm <= config_.gravity_norm_max;
    return result;
}

const Eigen::VectorXd &LtvObserver::state() const
{
    return state_;
}

const Eigen::MatrixXd &LtvObserver::covariance() const
{
    return covariance_;
}

int LtvObserver::slotForFeature(int feature_id) const
{
    const auto entry = feature_to_slot_.find(feature_id);
    return entry == feature_to_slot_.end() ? -1 : entry->second;
}

} // namespace ltv
