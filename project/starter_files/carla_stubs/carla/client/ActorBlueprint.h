#pragma once
#include <string>
namespace carla { namespace client {
struct ActorAttribute {};
class ActorBlueprint {
public:
    std::string GetId() const { return ""; }
};
}} // namespace
