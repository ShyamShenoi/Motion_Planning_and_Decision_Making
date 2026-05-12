#pragma once
#include <vector>
#include <memory>
#include "../geom/Transform.h"
#include "../road/RoadTypes.h"
#include <boost/shared_ptr.hpp>

namespace carla { namespace client {

class Waypoint {
public:
    carla::geom::Transform GetTransform() const { return {}; }
    bool IsJunction() const { return false; }
    carla::road::JuncId GetJunctionId() const { return -1; }
    std::vector<boost::shared_ptr<Waypoint>> GetNext(float) const { return {}; }
    std::vector<boost::shared_ptr<Waypoint>> GetNextUntilLaneEnd(float) const { return {}; }
};

}} // namespace carla::client
