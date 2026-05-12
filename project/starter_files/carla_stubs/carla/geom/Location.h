#pragma once
#include "Vector3D.h"

namespace carla { namespace geom {

// Location IS a Vector3D — same layout, extra semantic meaning
struct Location : public Vector3D {
    Location() = default;
    Location(float x, float y, float z) : Vector3D(x, y, z) {}
    explicit Location(const Vector3D& v) : Vector3D(v) {}

    Location operator+(const Location& o) const { return Location(x+o.x, y+o.y, z+o.z); }
    Location operator-(const Location& o) const { return Location(x-o.x, y-o.y, z-o.z); }
    bool operator==(const Location& o) const { return x==o.x && y==o.y && z==o.z; }
};

}} // namespace carla::geom
