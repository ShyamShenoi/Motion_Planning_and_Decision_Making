#pragma once

namespace carla { namespace geom {

struct Rotation {
    float pitch = 0.0f;
    float yaw   = 0.0f;
    float roll  = 0.0f;

    Rotation() = default;
    Rotation(float p, float y, float r) : pitch(p), yaw(y), roll(r) {}
};

}} // namespace carla::geom
