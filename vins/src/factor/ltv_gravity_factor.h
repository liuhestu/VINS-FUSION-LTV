#pragma once

#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

class LtvGravityFactor final : public ceres::SizedCostFunction<3, 7>
{
  public:
    LtvGravityFactor(const Eigen::Vector3d &gravity_body,
                     const Eigen::Vector3d &gravity_world,
                     double sigma_radians);

    bool Evaluate(double const *const *parameters,
                  double *residuals,
                  double **jacobians) const override;

    static bool computeResidual(const Eigen::Quaterniond &rotation_world_body,
                                const Eigen::Vector3d &gravity_body,
                                const Eigen::Vector3d &gravity_world,
                                double inverse_sigma,
                                Eigen::Vector3d &residual,
                                Eigen::Vector3d *vins_gravity_direction = nullptr);

  private:
    Eigen::Vector3d ltv_gravity_direction_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d world_gravity_direction_ = Eigen::Vector3d::Zero();
    double inverse_sigma_ = 0.0;
    bool valid_ = false;
};
