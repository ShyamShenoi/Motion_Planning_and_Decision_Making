#pragma once
#include "Map.h"
#include <boost/shared_ptr.hpp>

namespace carla { namespace client {

class World {
public:
    boost::shared_ptr<Map> GetMap() const { return boost::shared_ptr<Map>(); }
};

}} // namespace carla::client
