#include "ltv_velocity_factor.h"

#include "../utility/utility.h"

#include <cmath>

namespace
{

constexpr double kMinimumNorm = 1e-12;

bool validVector(const Eigen::Vector3d &value)
{
    return value.allFinite();
}

} // namespace

LtvVelocityFactor::LtvVelocityFactor(const Eigen::Vector3d &velocity_body,
                                     double sigma_mps)
{
    valid_ = validVector(velocity_body) && std::isfinite(sigma_mps) && sigma_mps > 0.0;
    if (!valid_)
        return;

    ltv_velocity_body_ = velocity_body;
    inverse_sigma_ = 1.0 / sigma_mps;
}

bool LtvVelocityFactor::computeResidual(
    const Eigen::Quaterniond &rotation_world_body,
    const Eigen::Vector3d &velocity_world,
    const Eigen::Vector3d &velocity_body,
    double inverse_sigma,
    Eigen::Vector3d &residual,
    Eigen::Vector3d *vins_velocity_body)
{
    if (!rotation_world_body.coeffs().allFinite() ||
        rotation_world_body.norm() <= kMinimumNorm ||
        !validVector(velocity_world) || !validVector(velocity_body) ||
        !std::isfinite(inverse_sigma) || inverse_sigma <= 0.0)
    {
        return false;
    }

    const Eigen::Quaterniond normalized_rotation = rotation_world_body.normalized();
    const Eigen::Vector3d velocity_body_vins =
        normalized_rotation.inverse() * velocity_world;
    residual = inverse_sigma * (velocity_body_vins - velocity_body);
    if (vins_velocity_body != nullptr)
        *vins_velocity_body = velocity_body_vins;
    return residual.allFinite();
}

bool LtvVelocityFactor::Evaluate(double const *const *parameters,
                                 double *residuals,
                                 double **jacobians) const
{
    if (!valid_)
        return false;

    const Eigen::Quaterniond rotation_world_body(
        parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    const Eigen::Vector3d velocity_world(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector3d residual;
    Eigen::Vector3d velocity_body_vins;
    if (!computeResidual(rotation_world_body, velocity_world, ltv_velocity_body_,
                         inverse_sigma_, residual, &velocity_body_vins))
    {
        return false;
    }

    Eigen::Map<Eigen::Vector3d> mapped_residual(residuals);
    mapped_residual = residual;
    if (jacobians != nullptr && jacobians[0] != nullptr)
    {
        // PoseLocalParameterization uses a right perturbation and maps its
        // tangent coordinates through the first six ambient columns.
        Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> pose_jacobian(jacobians[0]);
        pose_jacobian.setZero();
        pose_jacobian.block<3, 3>(0, 3) =
            inverse_sigma_ * Utility::skewSymmetric(velocity_body_vins);
    }
    if (jacobians != nullptr && jacobians[1] != nullptr)
    {
        Eigen::Map<Eigen::Matrix<double, 3, 9, Eigen::RowMajor>> speed_bias_jacobian(
            jacobians[1]);
        speed_bias_jacobian.setZero();
        speed_bias_jacobian.leftCols<3>() =
            inverse_sigma_ * rotation_world_body.normalized().toRotationMatrix().transpose();
    }
    return true;
}
