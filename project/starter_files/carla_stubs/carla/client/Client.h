#pragma once
#include <string>
#include <boost/shared_ptr.hpp>
#include "Waypoint.h"
#include "Map.h"
#include "World.h"
#include "ActorBlueprint.h"
#include "BlueprintLibrary.h"
#include "Sensor.h"
#include "TimeoutException.h"
#include "../geom/Transform.h"
#include "../image/ImageIO.h"
#include "../image/ImageView.h"
#include "../sensor/data/Image.h"

namespace carla { namespace client {

class World;

class Client {
public:
    Client(const std::string& host, uint16_t port, size_t worker_threads = 0) {}
    World GetWorld() const;
    void SetTimeout(double seconds) {}
};

}} // namespace carla::client
