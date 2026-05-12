#pragma once
#include "ActorBlueprint.h"
#include <vector>
namespace carla { namespace client {
class BlueprintLibrary {
public:
    std::vector<ActorBlueprint> Filter(const std::string&) const { return {}; }
};
}} // namespace
