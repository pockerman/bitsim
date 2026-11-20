#include "bitsim/geometry/rotations.h"

#ifdef BITSIM_CHRONO

#include <cmath>

namespace bitsim
{
namespace geometry
{

chrono::ChMatrix33d rot_x(real_t angle)
{
    const real_t c = std::cos(angle);
    const real_t s = std::sin(angle);

    chrono::ChMatrix33d R;
    R << 1.0, 0.0, 0.0, //
        0.0, c, -s,     //
        0.0, s, c;
    return R;
}

chrono::ChMatrix33d rot_y(real_t angle)
{
    const real_t c = std::cos(angle);
    const real_t s = std::sin(angle);

    chrono::ChMatrix33d R;
    R << c, 0.0, s,    //
        0.0, 1.0, 0.0, //
        -s, 0.0, c;
    return R;
}

chrono::ChMatrix33d rot_z(real_t angle)
{
    const real_t c = std::cos(angle);
    const real_t s = std::sin(angle);

    chrono::ChMatrix33d R;
    R << c, -s, 0.0, //
        s, c, 0.0,   //
        0.0, 0.0, 1.0;
    return R;
}

bool is_rotation(const chrono::ChMatrix33d &R, real_t tolerance)
{
    const bool orthogonal = (R.transpose() * R - chrono::ChMatrix33d(1.0)).norm() < tolerance;
    return orthogonal && std::abs(R.determinant() - 1.0) < tolerance;
}

bool same_matrix(const chrono::ChMatrix33d &A, const chrono::ChMatrix33d &B, real_t tolerance)
{
    return (A - B).norm() < tolerance;
}

chrono::ChMatrix33d from_yaw_pitch_roll(const YawPitchRoll &angles)
{
    return rot_z(angles.yaw) * rot_y(angles.pitch) * rot_x(angles.roll);
}

YawPitchRoll to_yaw_pitch_roll(const chrono::ChMatrix33d &R)
{
    return {std::atan2(R(1, 0), R(0, 0)), -std::asin(R(2, 0)), std::atan2(R(2, 1), R(2, 2))};
}

} // namespace geometry
} // namespace bitsim

#endif // BITSIM_CHRONO
