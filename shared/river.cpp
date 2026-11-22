#include "river.hpp"
#include <algorithm>
#include <cmath>
#include <istream>
#include <ostream>

namespace
{
Vector3 Mix(Vector3 a, Vector3 b, double t) { return a + (b - a) * t; }
bool FinitePoint(Vector3 p)
{
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
           std::abs(p.x) <= 200000 && std::abs(p.y) <= 200000 && std::abs(p.z) <= 200000;
}
}

RiverDefinition RiverDefinition::Straight(double width, double length)
{
    RiverDefinition river;
    for (int bank = 0; bank < 2; ++bank)
    {
        const double x = (bank == 0 ? -.5 : .5) * width;
        river.banks[bank] = {{{x,0,0}, {}, {0,0,length/3}}, {{x,0,length}, {0,0,-length/3}, {}}};
    }
    return river;
}

Vector3 RiverDefinition::Evaluate(int bank, int segment, double t) const
{
    const auto& a = banks.at(bank).at(segment);
    const auto& b = banks.at(bank).at(segment+1);
    const double s = 1-t;
    return a.position*(s*s*s) + (a.position+a.outgoing)*(3*s*s*t) +
           (b.position+b.incoming)*(3*s*t*t) + b.position*(t*t*t);
}

int RiverDefinition::PointCount() const { return static_cast<int>(banks[0].size() + banks[1].size()) + 2; }
int RiverDefinition::End(int point) const
{
    const int end = point - (PointCount()-2);
    return end == 0 || end == 1 ? end : -1;
}
bool RiverDefinition::ConnectedPoint(int point) const
{
    if (!connection) return false;
    if (End(point)>=0) return End(point)==connection->end;
    const int left=static_cast<int>(banks[0].size());
    return connection->end ? point==left-1 || point==PointCount()-3 : point==0 || point==left;
}
Vector3 RiverDefinition::Point(int point) const
{
    if (End(point) == 0) return (banks[0].front().position + banks[1].front().position)*.5;
    if (End(point) == 1) return (banks[0].back().position + banks[1].back().position)*.5;
    const int left = static_cast<int>(banks[0].size());
    return point < left ? banks[0].at(point).position : banks[1].at(point-left).position;
}

void RiverDefinition::SmoothHandles(int bank)
{
    auto& points = banks[bank];
    for (std::size_t i = 0; i < points.size(); ++i)
    {
        const auto p = points[i].position;
        const auto before = i ? points[i-1].position : p;
        const auto after = i+1 < points.size() ? points[i+1].position : p;
        const auto tangent = Math::Normalize(after-before);
        const double length = i == 0 ? Math::Length(after-p) : i+1 == points.size() ? Math::Length(p-before) :
            std::min(Math::Length(p-before), Math::Length(after-p));
        points[i].incoming = i ? tangent*(-length/3) : Vector3{};
        points[i].outgoing = i+1 < points.size() ? tangent*(length/3) : Vector3{};
    }
}

bool RiverDefinition::MovePoint(int point, Vector3 position)
{
    if (point < 0 || point >= PointCount() || !FinitePoint(position)) return SDL_SetError("Invalid river point");
    if (ConnectedPoint(point)) return SDL_SetError("Disconnect this river end before moving it");
    const int end = End(point);
    if (end >= 0)
    {
        const auto delta = position - Point(point);
        for (int bank = 0; bank < 2; ++bank)
        {
            auto& p = end ? banks[bank].back() : banks[bank].front();
            p.position = p.position + delta;
            SmoothHandles(bank);
        }
    }
    else
    {
        const int left = static_cast<int>(banks[0].size()), bank = point < left ? 0 : 1;
        banks[bank][bank ? point-left : point].position = position;
        SmoothHandles(bank);
    }
    return true;
}

bool RiverDefinition::TranslatePoints(const std::set<int>& indices, Vector3 delta)
{
    if (indices.empty()) return false;
    for (int point:indices) if (ConnectedPoint(point)) return SDL_SetError("Disconnect this river end before moving it");
    const int left = static_cast<int>(banks[0].size());
    std::array<std::size_t, 2> counts{};
    for (int i : indices)
    {
        if (i < 0 || i >= PointCount()-2 || !FinitePoint(Point(i) + delta))
            return SDL_SetError("Invalid river point translation");
        ++counts[i < left ? 0 : 1];
    }
    if (delta == Vector3{}) return true;
    for (int i : indices)
    {
        auto& point = i < left ? banks[0][i] : banks[1][i-left];
        point.position = point.position + delta;
    }
    for (int bank = 0; bank < 2; ++bank)
        if (counts[bank] && counts[bank] != banks[bank].size()) SmoothHandles(bank);
    return true;
}

bool RiverDefinition::Extend(int end, Vector3 position)
{
    if ((end != 0 && end != 1) || !FinitePoint(position)) return SDL_SetError("Invalid river endpoint");
    if (connection && connection->end==end) return SDL_SetError("Disconnect this river end before extending it");
    const auto delta = position - Point(PointCount()-2+end);
    if (std::hypot(delta.x, delta.z) < .5) return SDL_SetError("Extend the river by at least 0.5 m horizontally");
    for (const auto& bank : banks)
        if (bank.size() >= LakeDefinition::maxPoints) return SDL_SetError("River bank control point limit reached");
    for (int bank = 0; bank < 2; ++bank)
    {
        const auto p = (end ? banks[bank].back() : banks[bank].front()).position + delta;
        banks[bank].insert(end ? banks[bank].end() : banks[bank].begin(), {p, {}, {}});
        SmoothHandles(bank);
    }
    return true;
}

int RiverDefinition::InsertPoint(int bank, int segment, double t)
{
    if (bank < 0 || bank > 1 || segment < 0 || static_cast<std::size_t>(segment+1) >= banks[bank].size() ||
        banks[bank].size() >= LakeDefinition::maxPoints || !std::isfinite(t) || t <= .001 || t >= .999)
    { SDL_SetError("Choose a position between river bank points"); return -1; }
    auto& points = banks[bank];
    auto& a = points[segment]; auto& b = points[segment+1];
    const auto p = Mix(a.position, a.position+a.outgoing, t), q = Mix(a.position+a.outgoing, b.position+b.incoming, t);
    const auto r = Mix(b.position+b.incoming, b.position, t), s = Mix(p,q,t), u = Mix(q,r,t), center = Mix(s,u,t);
    a.outgoing = p-a.position; b.incoming = r-b.position;
    points.insert(points.begin()+segment+1, {center, s-center, u-center});
    return segment+1 + (bank ? static_cast<int>(banks[0].size()) : 0);
}

bool RiverDefinition::RemovePoint(int point)
{
    if (ConnectedPoint(point)) return SDL_SetError("Disconnect this river end before deleting its bank points");
    if (point < 0 || point >= PointCount()-2) return SDL_SetError("Delete a bank control point, not a river end handle");
    const int left = static_cast<int>(banks[0].size()), bank = point < left ? 0 : 1;
    if (banks[bank].size() <= 2) return SDL_SetError("A river bank needs at least two control points");
    banks[bank].erase(banks[bank].begin() + (bank ? point-left : point));
    SmoothHandles(bank);
    return true;
}

bool RiverDefinition::Read(std::istream& in, bool elevations, bool connections)
{
    if (!(in >> depth)) return false;
    for (auto& bank : banks)
    {
        std::size_t count;
        if (!(in >> count) || count < 2 || count > LakeDefinition::maxPoints) return SDL_SetError("Invalid river bank size");
        bank.resize(count);
        for (auto& p : bank)
            for (auto* v : {&p.position, &p.incoming, &p.outgoing})
            {
                v->y = 0; // TERR 6/7 stored only X/Z.
                if (!(in >> v->x) || (elevations && !(in >> v->y)) || !(in >> v->z)) return false;
            }
    }
    connection.reset();
    if (connections)
    {
        bool linked;
        if (!(in >> linked)) return false;
        if (linked)
        {
            RiverConnection c;
            // A closed lake shoreline may attach at fraction zero. Open river
            // end clearance is validated when the connection is resolved.
            if (!(in >> c.waterId >> c.end >> c.bank >> c.fraction >> c.width) || !c.waterId ||
                c.end < 0 || c.end > 1 || c.bank < 0 || c.bank > 1 ||
                !std::isfinite(c.fraction) || c.fraction < 0 || c.fraction >= 1 ||
                !std::isfinite(c.width) || c.width < .5 || c.width > 10000) return false;
            connection = c;
        }
    }
    return true;
}
bool RiverDefinition::Write(std::ostream& out) const
{
    out << depth;
    for (const auto& bank : banks)
    {
        out << ' ' << bank.size();
        for (const auto& p : bank)
            for (const auto v : {p.position, p.incoming, p.outgoing}) out << ' ' << v.x << ' ' << v.y << ' ' << v.z;
    }
    out << ' ' << connection.has_value();
    if (connection)
    {
        const auto& c = *connection;
        out << ' ' << c.waterId << ' ' << c.end << ' ' << c.bank << ' ' << c.fraction << ' ' << c.width;
    }
    return bool(out);
}

std::optional<std::vector<LakeSample>> RiverGeometry::Sample(const RiverDefinition& river)
{
    const auto fail = [](const char* message) -> std::optional<std::vector<LakeSample>> { SDL_SetError("%s", message); return {}; };
    if (!std::isfinite(river.depth) || river.depth <= 0 || river.depth > 1000) return fail("Invalid river depth");
    double length = 0;
    for (const auto& bank : river.banks)
    {
        if (bank.size() < 2 || bank.size() > LakeDefinition::maxPoints) return fail("Invalid river bank size");
        for (std::size_t i = 0; i < bank.size(); ++i)
        {
            const auto& a = bank[i];
            if (!FinitePoint(a.position) || !FinitePoint(a.incoming) || !FinitePoint(a.outgoing)) return fail("River points must have finite coordinates");
            if (i+1 == bank.size()) continue;
            const auto& b = bank[i+1];
            if (std::hypot(a.position.x-b.position.x, a.position.z-b.position.z) < .5)
                return fail("River bank points must be at least 0.5 m apart horizontally");
            length += Math::Length(a.outgoing) + Math::Length(b.incoming) + Math::Length(b.position+b.incoming-a.position-a.outgoing);
        }
    }
    std::array<std::vector<LakeSample>, 2> samples;
    const double step = std::max(2.0, length/2048);
    for (int bank = 0; bank < 2; ++bank)
    {
        const auto& points = river.banks[bank];
        for (int i = 0; i+1 < static_cast<int>(points.size()); ++i)
        {
            const auto& a = points[i]; const auto& b = points[i+1];
            const double chord = Math::Length(a.outgoing) + Math::Length(b.incoming) + Math::Length(b.position+b.incoming-a.position-a.outgoing);
            const int steps = std::max(4, int(std::ceil(chord/step)));
            for (int j = 0; j < steps; ++j) samples[bank].push_back({river.Evaluate(bank,i,double(j)/steps), i, double(j)/steps, bank});
        }
        samples[bank].push_back({points.back().position, static_cast<int>(points.size())-2, 1, bank});
    }
    auto rim = std::move(samples[0]);
    rim.insert(rim.end(), samples[1].rbegin(), samples[1].rend());
    rim.push_back(rim.front());
    return rim;
}
std::optional<LakeGeometry> RiverGeometry::Build(const RiverDefinition& river)
{
    auto rim = Sample(river);
    if (!rim) return {};
    return LakeGeometry::BuildShoreline(std::move(*rim), river.depth);
}

int WaterPointCount(const WaterDefinition& water)
{
    if (const auto* river = std::get_if<RiverDefinition>(&water)) return river->PointCount();
    return static_cast<int>(std::get<LakeDefinition>(water).points.size());
}
Vector3 WaterPoint(const WaterDefinition& water, int point)
{
    if (const auto* river = std::get_if<RiverDefinition>(&water)) return river->Point(point);
    return std::get<LakeDefinition>(water).points.at(point).position;
}
bool MoveWaterPoint(WaterDefinition& water, int point, Vector3 position)
{ return std::visit([&](auto& source) { return source.MovePoint(point, position); }, water); }
int InsertWaterPoint(WaterDefinition& water, int bank, int segment, double t)
{
    if (auto* river = std::get_if<RiverDefinition>(&water)) return river->InsertPoint(bank, segment, t);
    return std::get<LakeDefinition>(water).InsertPoint(segment, t) ? segment+1 : -1;
}
bool RemoveWaterPoint(WaterDefinition& water, int point)
{ return std::visit([&](auto& source) { return source.RemovePoint(point); }, water); }

std::optional<std::set<int>> WaterBankPoints(const WaterDefinition& water, const std::set<int>& points)
{
    if (points.empty()) return {};
    std::set<int> result;
    const auto* river = std::get_if<RiverDefinition>(&water);
    for (int point : points)
    {
        if (point < 0 || point >= WaterPointCount(water)) return {};
        const int end = river ? river->End(point) : -1;
        if (end < 0) result.insert(point);
        else
        {
            const int left = static_cast<int>(river->banks[0].size());
            result.insert(end ? left-1 : 0);
            result.insert(end ? river->PointCount()-3 : left);
        }
    }
    return result;
}

std::optional<Vector3> WaterSelectionCenter(const WaterDefinition& water, const std::set<int>& points)
{
    const auto bankPoints = WaterBankPoints(water, points);
    if (!bankPoints) return {};
    Vector3 center{};
    for (int point : *bankPoints) center = center + WaterPoint(water, point);
    return center * (1.0 / bankPoints->size());
}

bool TranslateWaterPoints(WaterDefinition& water, const std::set<int>& points, Vector3 delta)
{
    const auto bankPoints = WaterBankPoints(water, points);
    if (!bankPoints) return SDL_SetError("Invalid water control point selection");
    return std::visit([&](auto& source) { return source.TranslatePoints(*bankPoints, delta); }, water);
}

bool RemoveWaterPoints(WaterDefinition& water, const std::set<int>& points)
{
    if (points.empty()) return false;
    // Work on a copy: a failed minimum-point check must not partially delete a group.
    auto edited = water;
    for (auto it = points.rbegin(); it != points.rend(); ++it)
        if (!RemoveWaterPoint(edited, *it)) return false;
    water = std::move(edited);
    return true;
}
