#pragma once
#include "Waypoint.h"
#include "../geom/Location.h"
#include <boost/shared_ptr.hpp>

namespace carla { namespace client {

class Map {
public:
    boost::shared_ptr<Waypoint> GetWaypoint(
        const geom::Location&, bool project_to_road = true, int32_t lane_type = 0x1) const {
        return boost::shared_ptr<Waypoint>();
    }
};

}} // namespace carla::client
