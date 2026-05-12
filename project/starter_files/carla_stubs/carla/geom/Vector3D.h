#pragma once
#include <cmath>

namespace carla { namespace geom {

struct Vector3D {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    Vector3D() = default;
    Vector3D(float x, float y, float z) : x(x), y(y), z(z) {}

    Vector3D operator+(const Vector3D& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vector3D operator-(const Vector3D& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vector3D operator*(float s) const { return {x*s, y*s, z*s}; }
    bool operator==(const Vector3D& o) const { return x==o.x && y==o.y && z==o.z; }
    float SquaredLength() const { return x*x + y*y + z*z; }
    float Length() const { return std::sqrt(SquaredLength()); }
};

}} // namespace carla::geom
