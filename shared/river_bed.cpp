#include "river_bed.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace
{
    double Distance(Vector3 a, Vector3 b) { return std::hypot(a.x-b.x, a.z-b.z); }

    double BankDistance(Vector3 p, const std::vector<Vector3>& bank)
    {
        double result = std::numeric_limits<double>::max();
        for (std::size_t i = 1; i < bank.size(); ++i)
        {
            const auto a = bank[i-1], d = bank[i]-a;
            const double length2 = d.x*d.x+d.z*d.z;
            const double t = length2 > 0 ? std::clamp(((p.x-a.x)*d.x+(p.z-a.z)*d.z)/length2, 0.0, 1.0) : 0;
            result = std::min(result, Distance(p, a+d*t));
        }
        return result;
    }
}

RiverBedProfile::RiverBedProfile(std::span<const LakeSample> rim)
{
    // The closed polygon walks bank 0 forward, then bank 1 backward. Exclude
    // the repeated closing vertex; both polylines must run from start to end.
    for (std::size_t i = 0; i+1 < rim.size(); ++i)
        if (rim[i].bank >= 0 && rim[i].bank < 2) banks[rim[i].bank].push_back(rim[i].position);
    std::reverse(banks[1].begin(), banks[1].end());
}

std::vector<Vector3> RiverBedProfile::InteriorSamples() const
{
    std::array<std::vector<double>, 2> distances;
    std::vector<double> stations;
    for (int side = 0; side < 2; ++side)
    {
        if (banks[side].size() < 2) return {};
        auto& lengths = distances[side];
        lengths.push_back(0);
        for (std::size_t i = 1; i < banks[side].size(); ++i)
            lengths.push_back(lengths.back()+Distance(banks[side][i-1], banks[side][i]));
        if (lengths.back() <= 1e-8) return {};
        for (const double length : lengths) stations.push_back(length/lengths.back());
    }
    std::sort(stations.begin(), stations.end());
    stations.erase(std::unique(stations.begin(), stations.end(), [](double a, double b) { return b-a < 1e-9; }), stations.end());
    const auto at = [&](int side, double fraction) {
        const auto& lengths = distances[side];
        const double along = fraction*lengths.back();
        const auto next = std::upper_bound(lengths.begin(), lengths.end(), along);
        if (next == lengths.end()) return banks[side].back();
        const auto i = static_cast<std::size_t>(next-lengths.begin());
        return banks[side][i-1]+(banks[side][i]-banks[side][i-1])*((along-lengths[i-1])/(lengths[i]-lengths[i-1]));
    };
    std::vector<Vector3> points;
    std::set<std::pair<double,double>> used;
    // Three main regions: left bank, central channel, right bank. Three steps
    // on each slope round its top and toe; the middle third stays at full depth.
    constexpr std::array fractions{1./9, 2./9, 1./3, .5, 2./3, 7./9, 8./9};
    for (const double station : stations)
    {
        if (station <= 1e-9 || station >= 1-1e-9) continue; // Keep end-cap vertices at bank height.
        const auto left = at(0, station), right = at(1, station);
        for (const double fraction : fractions)
        {
            const auto p = left+(right-left)*fraction;
            if (used.emplace(std::round(p.x*1e7), std::round(p.z*1e7)).second) points.push_back(p);
        }
    }
    return points;
}

double RiverBedProfile::Depth(Vector3 p, double maximum, double shoreDistance) const
{
    if (shoreDistance <= 0) return 0;
    // Distances to the banks also work on bends and unequal bank point counts;
    // no authored cross-section pairing is required. Using the union's exposed
    // shore distance keeps internal banks and tributary mouths submerged.
    const double width = BankDistance(p, banks[0])+BankDistance(p, banks[1]);
    const double t = std::clamp(3*shoreDistance/std::max(width, 1e-8), 0.0, 1.0);
    return maximum*t*t*(3-2*t);
}
