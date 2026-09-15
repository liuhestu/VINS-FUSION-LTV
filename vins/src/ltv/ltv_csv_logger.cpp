#include "ltv_csv_logger.h"

#include <cstdlib>
#include <iomanip>

namespace ltv
{
namespace
{

std::string expandHome(const std::string &path)
{
    if (path.size() >= 2 && path[0] == '~' && path[1] == '/')
    {
        const char *home = std::getenv("HOME");
        if (home != nullptr)
            return std::string(home) + path.substr(1);
    }
    return path;
}

} // namespace

void LtvCsvLogger::configure(bool enabled, const std::string &path)
{
    close();
    if (!enabled || path.empty())
        return;

    stream_.open(expandHome(path), std::ios::out | std::ios::trunc);
    if (stream_)
    {
        stream_ << "frame_timestamp,imu_timestamp,state_features,observed_features,"
                   "velocity_body_x,velocity_body_y,velocity_body_z,"
                   "gravity_body_x,gravity_body_y,gravity_body_z,gravity_norm,"
                   "innovation_norm,covariance_trace,covariance_min_diagonal,"
                   "covariance_max_diagonal,healthy_camera_updates,valid,"
                   "velocity_valid,gravity_valid,update_time_ms,camera_substeps,reset_reason,"
                   "snapshot_time_error,gravity_angle_ltv_vs_vins_deg,"
                   "gravity_factor_residual_norm,gravity_factor_weighted_residual_norm,"
                   "gravity_factor_added\n";
    }
}

void LtvCsvLogger::close()
{
    if (stream_.is_open())
        stream_.close();
}

void LtvCsvLogger::write(const LtvSnapshot &snapshot)
{
    if (!stream_)
        return;

    stream_ << std::setprecision(16)
            << snapshot.frame_timestamp << ',' << snapshot.imu_timestamp << ','
            << snapshot.state_features << ',' << snapshot.observed_features << ','
            << snapshot.velocity_body.x() << ',' << snapshot.velocity_body.y() << ','
            << snapshot.velocity_body.z() << ',' << snapshot.gravity_body.x() << ','
            << snapshot.gravity_body.y() << ',' << snapshot.gravity_body.z() << ','
            << snapshot.gravity_body.norm() << ',' << snapshot.innovation_norm << ','
            << snapshot.covariance_trace << ',' << snapshot.covariance_min_diagonal << ','
            << snapshot.covariance_max_diagonal << ',' << snapshot.healthy_camera_updates << ','
            << snapshot.valid << ',' << snapshot.velocity_valid << ','
            << snapshot.gravity_valid << ',' << snapshot.update_time_ms << ','
            << snapshot.camera_substeps << ','
            << toString(snapshot.last_reset_reason) << ','
            << snapshot.snapshot_time_error << ','
            << snapshot.gravity_angle_ltv_vs_vins << ','
            << snapshot.gravity_factor_residual_norm << ','
            << snapshot.gravity_factor_weighted_residual_norm << ','
            << snapshot.gravity_factor_added << '\n';
    stream_.flush();
}

bool LtvCsvLogger::isOpen() const
{
    return stream_.is_open();
}

} // namespace ltv
