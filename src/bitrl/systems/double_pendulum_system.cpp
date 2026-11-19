#include "bitrl/bitrl_config.h"

#ifdef BITRL_CHRONO

#include "bitrl/bitrl_consts.h"
#include "bitrl/systems/double_pendulum_system.h"

#include <chrono/assets/ChVisualShapeBox.h>
#include <chrono/core/ChQuaternion.h>
#include <chrono/physics/ChLinkLock.h>

#include <cmath>
#include <stdexcept>

namespace bitrl
{
namespace systems
{

namespace
{
// A rod along its local z axis, with its pivot end at local z = +length / 2, so that the rotation
// by -theta about y points it at theta from the downward vertical
std::shared_ptr<chrono::ChBody> make_link(real_t length, real_t mass, real_t width)
{
    auto link = chrono_types::make_shared<chrono::ChBody>();
    link->SetMass(mass);
    const real_t across = mass * length * length / 12.0;
    link->SetInertiaXX(chrono::ChVector3d(across, across, mass * width * width / 6.0));
    link->AddVisualShape(chrono_types::make_shared<chrono::ChVisualShapeBox>(width, width, length));
    return link;
}

chrono::ChVector3d down_the_link(real_t theta)
{
    return chrono::ChVector3d(std::sin(theta), 0.0, -std::cos(theta));
}

// A revolute joint at p about the world's y axis: the joint turns about the z axis of its frame
void add_joint(chrono::ChSystemNSC &system, std::shared_ptr<chrono::ChBody> a,
               std::shared_ptr<chrono::ChBody> b, const chrono::ChVector3d &p)
{
    auto joint = chrono_types::make_shared<chrono::ChLinkLockRevolute>();
    joint->Initialize(a, b, chrono::ChFramed(p, chrono::QuatFromAngleX(-consts::maths::PI / 2.0)));
    system.AddLink(joint);
}

// The angle from the downward vertical of a link built by make_link
real_t link_angle(const chrono::ChBody &link)
{
    const chrono::ChVector3d d = link.GetRot().Rotate(chrono::ChVector3d(0.0, 0.0, -1.0));
    return std::atan2(d.x(), -d.z());
}
} // namespace

DoublePendulumSystem::DoublePendulumSystem(const DoublePendulumProperties &props, real_t theta1,
                                           real_t theta2)
    : props_(props)
{
    if (!(props.l1 > 0.0 && props.l2 > 0.0 && props.m1 > 0.0 && props.m2 > 0.0 &&
          props.width > 0.0))
    {
        throw std::invalid_argument(
            "DoublePendulumSystem: the lengths, masses and width must be positive");
    }
    system_.SetGravitationalAcceleration(chrono::ChVector3d(0.0, 0.0, -consts::maths::G));

    auto ground = chrono_types::make_shared<chrono::ChBody>();
    ground->SetFixed(true);
    system_.AddBody(ground);

    link1_ = make_link(props.l1, props.m1, props.width);
    link1_->SetPos(down_the_link(theta1) * (0.5 * props.l1));
    link1_->SetRot(chrono::QuatFromAngleY(-theta1));
    system_.AddBody(link1_);

    const chrono::ChVector3d elbow = down_the_link(theta1) * props.l1;
    link2_ = make_link(props.l2, props.m2, props.width);
    link2_->SetPos(elbow + down_the_link(theta2) * (0.5 * props.l2));
    link2_->SetRot(chrono::QuatFromAngleY(-theta2));
    system_.AddBody(link2_);

    add_joint(system_, link1_, ground, chrono::ChVector3d(0.0, 0.0, 0.0));
    add_joint(system_, link2_, link1_, elbow);
}

void DoublePendulumSystem::step(real_t dt)
{
    if (!(dt > 0.0))
    {
        throw std::invalid_argument("DoublePendulumSystem: dt must be positive");
    }
    system_.DoStepDynamics(dt);
}

std::array<real_t, 2> DoublePendulumSystem::angles() const
{
    return {link_angle(*link1_), link_angle(*link2_)};
}

std::array<real_t, 2> DoublePendulumSystem::angular_velocities() const
{
    // Turning about +y by w moves the link towards -x: the angle changes by -w
    return {-link1_->GetAngVelParent().y(), -link2_->GetAngVelParent().y()};
}

real_t DoublePendulumSystem::energy() const
{
    real_t e = 0.0;
    for (const auto &link : {link1_, link2_})
    {
        const chrono::ChVector3d w = link->GetAngVelLocal();
        const chrono::ChVector3d I = link->GetInertiaXX();
        e += 0.5 * link->GetMass() * link->GetPosDt().Length2() +
             0.5 * (I.x() * w.x() * w.x() + I.y() * w.y() * w.y() + I.z() * w.z() * w.z()) +
             link->GetMass() * consts::maths::G * link->GetPos().z();
    }
    return e;
}

} // namespace systems
} // namespace bitrl

#endif
