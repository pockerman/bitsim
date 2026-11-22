#include "bitsim/distributed/chrono_ghosts.h"

#ifdef BITSIM_CHRONO

#include <chrono/physics/ChContactMaterialNSC.h>

#include <set>

namespace bitsim
{
namespace distributed
{

BodyState chrono_body_state(const std::string &name, const chrono::ChBody &body,
                            const std::array<real_t, 3> &size)
{
    BodyState b;
    b.name = name;
    const auto &p = body.GetPos();
    const auto &q = body.GetRot();
    const auto &v = body.GetPosDt();
    const auto w = body.GetAngVelParent();
    b.position = {p.x(), p.y(), p.z()};
    b.orientation = {q.e0(), q.e1(), q.e2(), q.e3()};
    b.linear_velocity = {v.x(), v.y(), v.z()};
    b.angular_velocity = {w.x(), w.y(), w.z()};
    b.size = size;
    return b;
}

CHRONO_Ghosts::~CHRONO_Ghosts()
{
    for (auto &[key, ghost] : ghosts_)
    {
        system_.RemoveBody(ghost);
    }
}

std::shared_ptr<chrono::ChBody> CHRONO_Ghosts::get(const std::string &participant,
                                                   const std::string &body) const
{
    const auto itr = ghosts_.find(participant + "/" + body);
    return itr == ghosts_.end() ? nullptr : itr->second;
}

void CHRONO_Ghosts::update(const std::vector<PeerState> &peers)
{
    std::set<std::string> seen;
    for (const auto &peer : peers)
    {
        if (peer.left)
        {
            continue; // its ghosts go below
        }
        for (const auto &body : peer.bodies)
        {
            if (body.size[0] <= 0.0 || body.size[1] <= 0.0 || body.size[2] <= 0.0)
            {
                continue;
            }
            const std::string key = peer.participant + "/" + body.name;
            seen.insert(key);
            auto &ghost = ghosts_[key];
            if (!ghost)
            {
                auto material = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
                ghost = chrono_types::make_shared<chrono::ChBodyEasyBox>(
                    body.size[0], body.size[1], body.size[2], 1000.0, true, true, material);
                ghost->SetFixed(true);
                ghost->SetName(("ghost:" + key).c_str());
                ghost->GetVisualShape(0)->SetColor(chrono::ChColor(0.6f, 0.6f, 0.9f));
                system_.AddBody(ghost);
            }
            ghost->SetPos(chrono::ChVector3d(body.position[0], body.position[1], body.position[2]));
            ghost->SetRot(chrono::ChQuaterniond(body.orientation[0], body.orientation[1],
                                                body.orientation[2], body.orientation[3]));
        }
    }
    // the bodies of peers that left, or that no longer report a body
    for (auto itr = ghosts_.begin(); itr != ghosts_.end();)
    {
        if (!seen.contains(itr->first))
        {
            system_.RemoveBody(itr->second);
            itr = ghosts_.erase(itr);
        }
        else
        {
            ++itr;
        }
    }
}

} // namespace distributed
} // namespace bitsim

#endif
