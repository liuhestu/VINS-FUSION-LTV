#pragma once

#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

class LtvVelocityFactor final : public ceres::SizedCostFunction<3, 7, 9>
{
  public:
    LtvVelocityFactor(const Eigen::Vector3d &velocity_body, double sigma_mps);

    bool Evaluate(double const *const *parameters,
                  double *residuals,
                  double **jacobians) const override;

    static bool computeResidual(const Eigen::Quaterniond &rotation_world_body,
                                const Eigen::Vector3d &velocity_world,
                                const Eigen::Vector3d &velocity_body,
                                double inverse_sigma,
                                Eigen::Vector3d &residual,
                                Eigen::Vector3d *vins_velocity_body = nullptr);

  private:
    Eigen::Vector3d ltv_velocity_body_ = Eigen::Vector3d::Zero();
    double inverse_sigma_ = 0.0;
    bool valid_ = false;
};
