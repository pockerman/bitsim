#include "bitsim/estimation/diff_drive_odometry.h"
#include "bitsim/bitsim_consts.h"

#include <cmath>
#include <stdexcept>

namespace bitsim
{
namespace estimation
{

DiffDriveOdometry::DiffDriveOdometry(real_t wheel_radius, real_t axle_length)
    : wheel_radius_(wheel_radius), axle_length_(axle_length)
{
    if (wheel_radius_ <= 0.0 || axle_length_ <= 0.0)
    {
        throw std::logic_error("DiffDriveOdometry needs a positive wheel radius and axle length");
    }
}

void DiffDriveOdometry::reset(const Pose &pose, real_t left_angle, real_t right_angle)
{
    pose_ = pose;
    left_ = left_angle;
    right_ = right_angle;
    distance_ = 0.0;
}

real_t DiffDriveOdometry::step_(real_t left_angle, real_t right_angle, real_t &turned)
{
    const real_t left = wheel_radius_ * (left_angle - left_);
    const real_t right = wheel_radius_ * (right_angle - right_);
    left_ = left_angle;
    right_ = right_angle;

    turned = (right - left) / axle_length_;
    const real_t driven = 0.5 * (left + right);
    distance_ += std::abs(driven);
    return driven;
}

const DiffDriveOdometry::Pose &DiffDriveOdometry::update(real_t left_angle, real_t right_angle)
{
    real_t turned = 0.0;
    const real_t driven = step_(left_angle, right_angle, turned);
    const real_t middle = pose_.yaw + 0.5 * turned;
    pose_.x += driven * std::cos(middle);
    pose_.y += driven * std::sin(middle);
    pose_.yaw = std::remainder(pose_.yaw + turned, 2.0 * consts::maths::PI);
    return pose_;
}

const DiffDriveOdometry::Pose &DiffDriveOdometry::update(real_t left_angle, real_t right_angle,
                                                         real_t heading)
{
    real_t turned = 0.0;
    const real_t driven = step_(left_angle, right_angle, turned);
    const real_t middle =
        pose_.yaw + 0.5 * std::remainder(heading - pose_.yaw, 2.0 * consts::maths::PI);
    pose_.x += driven * std::cos(middle);
    pose_.y += driven * std::sin(middle);
    pose_.yaw = std::remainder(heading, 2.0 * consts::maths::PI);
    return pose_;
}

} // namespace estimation
} // namespace bitsim
