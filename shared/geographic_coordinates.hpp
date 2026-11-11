#pragma once

#include "math.hpp"
#include <optional>

// WGS 84 decimal degrees in the order copied by Google Maps.
struct GeographicCoordinates
{
    double latitude, longitude;
    static constexpr double originLatitude = 40.0;
    static constexpr double originLongitude = -98.5;
    static constexpr double realMetersPerWorldMeter = 6.0;

    // +X is east, +Z is north. Elevation is already in game meters.
    std::optional<Vector3> ToWorld(double elevation) const;
    static std::optional<GeographicCoordinates> FromWorld(Vector3 position);
};
