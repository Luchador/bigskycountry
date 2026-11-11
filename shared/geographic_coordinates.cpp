#include "geographic_coordinates.hpp"
#include <cmath>

namespace
{
    constexpr double radians = std::numbers::pi / 180.0;
    constexpr double semiMajorAxis = 6378137.0; // WGS 84 meters
    constexpr double flattening = 1.0 / 298.257223563;
    const double eccentricity = std::sqrt(flattening * (2 - flattening));
    const double standardParallel = GeographicCoordinates::originLatitude * radians;
    const double eSinStandard = eccentricity * std::sin(standardParallel);
    const double parallelRadius = semiMajorAxis * std::cos(standardParallel) / std::sqrt(1 - eSinStandard * eSinStandard);

    // Ellipsoidal Mercator. Both axes have the same local scale, true at 40 N.
    double IsometricLatitude(double phi)
    {
        return std::asinh(std::tan(phi)) - eccentricity * std::atanh(eccentricity * std::sin(phi));
    }

    const double originNorthing = IsometricLatitude(standardParallel);
}

std::optional<Vector3> GeographicCoordinates::ToWorld(double elevation) const
{
    if (!(std::abs(latitude) < 90 && std::abs(longitude) <= 180) || !std::isfinite(elevation)) return {};
    const double deltaLongitude = std::remainder(longitude - originLongitude, 360.0);
    const double east = parallelRadius * (deltaLongitude * radians);
    const double north = parallelRadius * (IsometricLatitude(latitude * radians) - originNorthing);
    return Vector3{east / realMetersPerWorldMeter, elevation, north / realMetersPerWorldMeter};
}

std::optional<GeographicCoordinates> GeographicCoordinates::FromWorld(Vector3 position)
{
    const double deltaLongitude = (position.x * realMetersPerWorldMeter / parallelRadius) / radians;
    const double psi = originNorthing + position.z * realMetersPerWorldMeter / parallelRadius;
    if (!std::isfinite(psi) || !(std::abs(deltaLongitude) <= 180.0 + 1e-12)) return {};
    const double longitude = std::remainder(originLongitude + deltaLongitude, 360.0);

    // Invert isometric latitude. The eccentricity correction contracts by at most e squared.
    double phi = std::atan(std::sinh(psi));
    for (int i = 0; i < 8; ++i)
    {
        const double correction = eccentricity * std::atanh(eccentricity * std::sin(phi));
        const double next = std::atan(std::sinh(psi + correction));
        if (next == phi) break;
        phi = next;
    }
    return GeographicCoordinates{phi / radians, longitude};
}
