#include "bitsim/distributed/state_message.h"

#include "bitsim/extern/nlohmann/json/json.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>

namespace bitsim
{
namespace distributed
{

BodyState BodyState::planar(const std::string &name, real_t x, real_t y, real_t yaw, real_t vx,
                            real_t vy, real_t wz, const std::array<real_t, 3> &size)
{
    BodyState b;
    b.name = name;
    b.position = {x, y, 0.0};
    b.orientation = {std::cos(0.5 * yaw), 0.0, 0.0, std::sin(0.5 * yaw)};
    b.linear_velocity = {vx, vy, 0.0};
    b.angular_velocity = {0.0, 0.0, wz};
    b.size = size;
    return b;
}

real_t BodyState::yaw() const noexcept
{
    const auto &[w, x, y, z] = orientation;
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

BodyState BodyState::extrapolated(real_t dt) const
{
    BodyState b = *this;
    for (int i = 0; i < 3; ++i)
    {
        b.position[i] += linear_velocity[i] * dt;
    }

    // turn by the angular velocity, given in the world frame: q' = exp(w dt / 2) q
    const real_t wx = angular_velocity[0], wy = angular_velocity[1], wz = angular_velocity[2];
    const real_t rate = std::sqrt(wx * wx + wy * wy + wz * wz);
    if (rate * std::abs(dt) > 0.0)
    {
        const real_t half = 0.5 * rate * dt;
        const real_t s = std::sin(half) / rate;
        const std::array<real_t, 4> r{std::cos(half), wx * s, wy * s, wz * s};
        const auto &q = orientation;
        b.orientation = {r[0] * q[0] - r[1] * q[1] - r[2] * q[2] - r[3] * q[3],
                         r[0] * q[1] + r[1] * q[0] + r[2] * q[3] - r[3] * q[2],
                         r[0] * q[2] - r[1] * q[3] + r[2] * q[0] + r[3] * q[1],
                         r[0] * q[3] + r[1] * q[2] - r[2] * q[1] + r[3] * q[0]};
    }
    return b;
}

std::string StateMessage::encode() const
{
    nlohmann::json j;
    j["k"] = static_cast<uint_t>(kind);
    j["p"] = participant;
    j["s"] = step;
    j["t"] = time;
    j["w"] = stamp;
    j["b"] = nlohmann::json::array();
    for (const auto &body : bodies)
    {
        j["b"].push_back({{"n", body.name},
                          {"p", body.position},
                          {"q", body.orientation},
                          {"v", body.linear_velocity},
                          {"o", body.angular_velocity},
                          {"z", body.size}});
    }
    const auto bytes = nlohmann::json::to_msgpack(j);
    return std::string(bytes.begin(), bytes.end());
}

StateMessage StateMessage::decode(const std::string &bytes)
{
    try
    {
        const auto j = nlohmann::json::from_msgpack(bytes);
        StateMessage m;
        m.kind = static_cast<Kind>(j.at("k").get<uint_t>());
        m.participant = j.at("p").get<std::string>();
        m.step = j.at("s").get<uint_t>();
        m.time = j.at("t").get<real_t>();
        m.stamp = j.at("w").get<real_t>();
        for (const auto &b : j.at("b"))
        {
            BodyState body;
            body.name = b.at("n").get<std::string>();
            body.position = b.at("p").get<std::array<real_t, 3>>();
            body.orientation = b.at("q").get<std::array<real_t, 4>>();
            body.linear_velocity = b.at("v").get<std::array<real_t, 3>>();
            body.angular_velocity = b.at("o").get<std::array<real_t, 3>>();
            body.size = b.at("z").get<std::array<real_t, 3>>();
            m.bodies.push_back(std::move(body));
        }
        return m;
    }
    catch (const nlohmann::json::exception &e)
    {
        throw std::invalid_argument(std::string("StateMessage: not a state message: ") + e.what());
    }
}

real_t wall_time_now()
{
    return std::chrono::duration<real_t>(std::chrono::system_clock::now().time_since_epoch())
        .count();
}

} // namespace distributed
} // namespace bitsim
