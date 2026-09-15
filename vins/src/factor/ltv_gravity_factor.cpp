#include "ltv_gravity_factor.h"

#include "../utility/utility.h"

#include <cmath>

namespace
{

constexpr double kMinimumNorm = 1e-12;

bool validDirection(const Eigen::Vector3d &value)
{
    return value.allFinite() && value.norm() > kMinimumNorm;
}

} // namespace

LtvGravityFactor::LtvGravityFactor(const Eigen::Vector3d &gravity_body,
                                   const Eigen::Vector3d &gravity_world,
                                   double sigma_radians)
{
    valid_ = validDirection(gravity_body) && validDirection(gravity_world) &&
             std::isfinite(sigma_radians) && sigma_radians > 0.0;
    if (!valid_)
        return;

    ltv_gravity_direction_ = gravity_body.normalized();
    world_gravity_direction_ = gravity_world.normalized();
    inverse_sigma_ = 1.0 / sigma_radians;
}

bool LtvGravityFactor::computeResidual(
    const Eigen::Quaterniond &rotation_world_body,
    const Eigen::Vector3d &gravity_body,
    const Eigen::Vector3d &gravity_world,
    double inverse_sigma,
    Eigen::Vector3d &residual,
    Eigen::Vector3d *vins_gravity_direction)
{
    if (!rotation_world_body.coeffs().allFinite() ||
        rotation_world_body.norm() <= kMinimumNorm ||
        !validDirection(gravity_body) || !validDirection(gravity_world) ||
        !std::isfinite(inverse_sigma) || inverse_sigma <= 0.0)
    {
        return false;
    }

    const Eigen::Quaterniond normalized_rotation = rotation_world_body.normalized();
    const Eigen::Vector3d direction_vins =
        normalized_rotation.inverse() * gravity_world.normalized();
    residual = inverse_sigma * (direction_vins - gravity_body.normalized());
    if (vins_gravity_direction != nullptr)
        *vins_gravity_direction = direction_vins;
    return residual.allFinite();
}

bool LtvGravityFactor::Evaluate(double const *const *parameters,
                                double *residuals,
                                double **jacobians) const
{
    if (!valid_)
        return false;

    const Eigen::Quaterniond rotation_world_body(
        parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    Eigen::Vector3d residual;
    Eigen::Vector3d gravity_direction_vins;
    if (!computeResidual(rotation_world_body, ltv_gravity_direction_,
                         world_gravity_direction_, inverse_sigma_, residual,
                         &gravity_direction_vins))
    {
        return false;
    }

    Eigen::Map<Eigen::Vector3d> mapped_residual(residuals);
    mapped_residual = residual;
    if (jacobians != nullptr && jacobians[0] != nullptr)
    {
        // PoseLocalParameterization uses a right perturbation q_plus = q * dq
        // and expects local derivatives in the first six ambient columns.
        Eigen::Map<Eigen::Matrix<double, 3, 7, Eigen::RowMajor>> jacobian(jacobians[0]);
        jacobian.setZero();
        jacobian.block<3, 3>(0, 3) =
            inverse_sigma_ * Utility::skewSymmetric(gravity_direction_vins);
    }
    return true;
}
