#include "transform_inspector.hpp"
#include "world_document.hpp"
#include "world_renderer.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>

namespace
{
    constexpr double degrees = 180.0 / std::numbers::pi;

    double& Component(Vector3& v, int axis)
    {
        if (axis == 0)
        {
            return v.x;
        }

        if (axis == 1)
        {
            return v.y;
        }

        return v.z;
    }

    Vector3& Row(ObjectTransform& t, int field)
    {
        if (field < 3)
        {
            return t.position;
        }
        if (field < 6)
        {
            return t.rotation;
        }
        return t.scale;
    }
}

bool TransformFields::CanEdit(int field) const
{
    if (target.kind == SelectionKind::None || field < 0 || field >= 9)
    {
        return false;
    }

    if (target.kind == SelectionKind::Intersection) return field < 3;
    return field < 6 || target.kind == SelectionKind::Object;
}

double TransformFields::DisplayValue(int field) const
{
    ObjectTransform t = value;
    return Component(Row(t, field), field % 3) * (field >= 3 && field < 6 ? degrees : 1.0);
}

std::string TransformFields::Text(int field) const
{
    if (!CanEdit(field))
    {
        return {};
    }

    std::ostringstream out;
    out.imbue(std::locale::classic());
    const double n = DisplayValue(field);
    out << std::fixed << std::setprecision(2) << (std::abs(n) < 0.005 ? 0.0 : n);

    return out.str();
}

std::string TransformFields::Title() const
{
    if (!selectionLabel.empty()) return selectionLabel;
    const std::string n = std::to_string(target.index + 1);
    switch (target.kind)
    {
    case SelectionKind::Intersection: return "Intersection " + std::to_string(target.index);
    case SelectionKind::Road: return "Road " + n;
    case SelectionKind::Point: return "Road " + n + " / Point " + std::to_string(target.point + 1);
    case SelectionKind::Object: return "Object " + n;
    case SelectionKind::Terrain: return "Terrain " + n;
    default: return "No selection";
    }
}

std::optional<double> ParseTransformValue(std::string_view text)
{
    const auto start = text.find_first_not_of(" \t");

    if (start == text.npos)
    {
        return {};
    }

    text = text.substr(start, text.find_last_not_of(" \t") - start + 1);

    if (text.front() == '+')
    {
        text.remove_prefix(1);
        if (text.empty() || text.front() == '-')
        {
            return {};
        }
    }

    double value;
    const auto end = text.data() + text.size();
    const auto result = std::from_chars(text.data(), end, value);

    if (result.ec != std::errc{} || result.ptr != end || !std::isfinite(value))
    {
        return {};
    }

    return value;
}

TransformFields Inspect(const WorldDocument& world, TransformTarget target)
{
    TransformFields fields{target, {}};
    const auto i = static_cast<std::size_t>(target.index);
    switch (target.kind)
    {
    case SelectionKind::Road:
    case SelectionKind::Point:
    {
        if (i >= world.roads.size() || world.roads[i].points.empty())
        {
            return {};
        }
        const Road& road = world.roads[i];
        if (target.kind == SelectionKind::Road)
        {
            fields.value.position = road.points.front().position;
            fields.value.rotation = road.rotation;
        }
        else
        {
            if (static_cast<std::size_t>(target.point) >= road.points.size())
            {
                return {};
            }
            const auto axes = road.PointAxes(target.point, false);
            fields.value.rotation.x = road.points[target.point].roll;
            fields.value.position = road.points[target.point].position;
            fields.value.rotation.y = std::atan2(axes[2].x, axes[2].z);
            fields.value.rotation.z = std::atan2(axes[0].y, std::hypot(axes[0].x, axes[0].z));
        }
        break;
    }
    case SelectionKind::Intersection:
        if (!world.intersections.contains(target.index)) return {};
        fields.value.position = world.intersections.at(target.index).position;
        break;
    case SelectionKind::Terrain:
        if (i >= world.Terrain().size())
        {
            return {};
        }
        fields.value = world.Terrain()[i]->GetTransform();
        break;
    case SelectionKind::Object:
        if (i >= world.Objects().size())
        {
            return {};
        }
        fields.value = world.Objects()[i]->GetTransform();
        break;
    default: return {};
    }
    return fields;
}

bool SetPlacementTransform(WorldDocument& world, TransformTarget target, const ObjectTransform& transform)
{
    const auto index = static_cast<std::size_t>(target.index);
    if (target.kind == SelectionKind::Intersection && world.intersections.contains(target.index) &&
        std::isfinite(Math::Length(transform.position)) && transform.rotation == Vector3{} && transform.scale == Vector3{1,1,1})
    {
        world.intersections.at(target.index).position = transform.position;
        return true;
    }
    if (target.kind == SelectionKind::Object && index < world.Objects().size())
    {
        return world.Objects()[index]->SetTransform(transform);
    }
    if (target.kind == SelectionKind::Terrain && index < world.Terrain().size())
    {
        return world.Terrain()[index]->SetTransform(transform);
    }
    return SDL_SetError("Invalid mesh selection");
}

bool ApplyTransform(WorldDocument& world, const TransformFields& fields, int field, double value)
{
    if (!fields.CanEdit(field) || !std::isfinite(value))
    {
        return SDL_SetError("Invalid transform field");
    }

    ObjectTransform t = fields.value;
    Component(Row(t, field), field % 3) = field >= 3 && field < 6 ? value / degrees : value;
    const TransformTarget target = fields.target;

    if (target.kind == SelectionKind::Object || target.kind == SelectionKind::Terrain || target.kind == SelectionKind::Intersection)
    {
        return SetPlacementTransform(world, target, t);
    }

    Road& road = world.roads[target.index];

    if (target.kind == SelectionKind::Road)
    {
        return road.SetPose(t.position, t.rotation) || SDL_SetError("Invalid road transform");
    }

    const int i = target.point;
    RoadPoint& p = road.points[i];

    if (field < 3)
    {
        for (int neighbor : {i - 1, i + 1})
        {
            if (neighbor < 0 || neighbor >= static_cast<int>(road.points.size()))
            {
                continue;
            }

            const double span = Math::Length(t.position - road.points[neighbor].position);

            if (span < 0.05)
            {
                return SDL_SetError("Road points must be at least 0.05 m apart");
            }

            if (!std::isfinite(span))
            {
                return SDL_SetError("Road segment length is out of range");
            }

            // Each 12 m dash cycle needs at least one six-vertex quad per dashed divider.
            // The shared GPU vertex buffer has a 32-bit byte size.
            const int dividers = std::max(0, road.style.GetLaneCount() - 2);
            const double dashBytes = span / RoadStyle::dashPeriod * dividers * 6 * sizeof(WorldDepthVertex);

            if (dashBytes > std::numeric_limits<Uint32>::max())
            {
                return SDL_SetError("The dashed markings exceed the world vertex buffer limit");
            }
        }
        p.position = t.position;
    }
    else if (field == 3)
    {
        p.roll = t.rotation.x;
    }
    else
    {
        if (std::abs(t.rotation.z) > std::numbers::pi / 2)
        {
            return SDL_SetError("Point pitch must be between -90 and 90 degrees");
        }
        const double heading = -t.rotation.y;
        if (road.style.curved)
        {
            p.heading = heading;
            p.pitch = t.rotation.z;
        }
        else if (field == 4)
        {
            road.RotatePoint(i, {0, 1, 0}, t.rotation.y - fields.value.rotation.y);
        }
        else
        {
            road.RotatePoint(i, {-std::sin(heading), 0, std::cos(heading)}, t.rotation.z - fields.value.rotation.z);
        }
        const Vector3 tangent = road.Tangent(i);
        if (!road.style.curved && std::hypot(tangent.x, tangent.z) < 1.0e-8)
        {
            p.heading = heading;
            p.pitch = t.rotation.z;
        }
        p.roll = t.rotation.x;
    }

    road.Rebuild();

    return true;
}
