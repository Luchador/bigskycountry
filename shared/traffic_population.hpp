#pragma once

#include "camera_view.hpp"
#include "spatial_index.hpp"
#include "traffic_network.hpp"
#include <random>

// Chooses spawn sites from resident navigation metadata; never rebuilds roads or
// loads geometry. The session requests collision before publishing a car.
class TrafficPopulation
{
public:
    static constexpr std::size_t targetCars = 24;
    static constexpr double spawnNearMeters = 150, spawnFarMeters = 600;
    static constexpr double visibleSpawnMeters = 450;
    static constexpr double despawnMeters = 1000;
    static constexpr double attemptSeconds = 0.5, loadTimeoutSeconds = 5;
    static constexpr double runwayMeters = 40;
    struct Site { std::size_t lane; double distance; };

    explicit TrafficPopulation(const TrafficNetwork& network, std::uint32_t seed = std::random_device{}());
    std::optional<Site> Pick(Vector3 player, const CameraView& view);
    static bool InSpawnArea(Vector3 point, Vector3 player, const CameraView& view);

private:
    const TrafficNetwork& network;
    std::mt19937 random;
    struct Span { std::size_t lane; double first, last; };
    std::vector<Span> spans;
    SpatialIndex index;
};
