#pragma once
#include <stdexcept>
namespace carla { namespace client {
class TimeoutException : public std::runtime_error {
public:
    TimeoutException(const std::string& s = "") : std::runtime_error(s) {}
};
}} // namespace
