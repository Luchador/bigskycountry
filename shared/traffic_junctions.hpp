#pragma once
#include "traffic_driver.hpp"
#include <map>
#include <memory>

// One arbiter per session; all requests see the same pre-physics snapshot.
class TrafficJunctions
{
public:
    TrafficJunctions(GameWorld& world, const TrafficNetwork& network) : world(world), network(network) {}
    void Update(double seconds, const std::vector<std::unique_ptr<TrafficDriver>>& drivers);
    void Clear() { requests.clear(); clock = 0; }
    void Remove(PawnHandle handle) { requests.erase(handle); }
    std::size_t Reservations() const;
private:
    struct Request
    {
        TrafficDriver* driver = nullptr;
        const TrafficMovement* move = nullptr;
        unsigned visit = 0;
        double arrived = -1, stopped = 0, distance = 0, speed = 0;
        bool reserved = false, entered = false, eligible = false, exitClear = false;
    };
    GameWorld& world;
    const TrafficNetwork& network;
    std::map<PawnHandle, Request> requests;
    double clock = 0;
    bool Conflicts(const TrafficMovement& a, const TrafficMovement& b) const;
    bool ExitClear(const Request& request) const;
    bool Intruder(const Request& request) const;
};
