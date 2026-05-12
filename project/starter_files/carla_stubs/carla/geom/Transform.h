#pragma once
#include "Location.h"
#include "Rotation.h"

namespace carla { namespace geom {

struct Transform {
    Location location;
    Rotation rotation;

    Transform() = default;
    Transform(const Location& l, const Rotation& r) : location(l), rotation(r) {}
};

}} // namespace carla::geom
