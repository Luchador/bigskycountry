#include "object_io.hpp"
#include "vehicle.hpp"
#include "world_document.hpp"

#include <cmath>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <utility>

namespace ObjectIO
{
    bool Read(std::string payload, WorldDocument& world)
    {
        std::istringstream in(std::move(payload));
        in.imbue(std::locale::classic());
        in >> std::ws;
        while (!in.eof())
        {
            unsigned type;
            std::string path;
            ObjectTransform t;
            float health;
            bool invincible;
            if (!(in >> type >> std::quoted(path)
                     >> t.position.x >> t.position.y >> t.position.z
                     >> t.rotation.x >> t.rotation.y >> t.rotation.z
                     >> t.scale.x >> t.scale.y >> t.scale.z >> health >> invincible) ||
                !std::isfinite(health) || health < 0.0f)
            {
                return SDL_SetError("Invalid object record in OBJS");
            }

            const ObjectType kind = static_cast<ObjectType>(type);
            float acceleration = 0.0f;
            if (kind == ObjectType::Vehicle && (!(in >> acceleration) || !std::isfinite(acceleration)))
            {
                return SDL_SetError("Invalid vehicle acceleration in OBJS");
            }
            BSCObject* object = world.PlaceObject(kind, path, t);
            if (!object)
            {
                return false;
            }
            object->health = health;
            object->invincible = invincible;
            if (auto* vehicle = dynamic_cast<Vehicle*>(object))
            {
                vehicle->acceleration = acceleration;
            }
            in >> std::ws;
        }
        return true;
    }

    bool Write(std::ostream& out, const WorldDocument& world)
    {
        out.imbue(std::locale::classic());
        out << std::setprecision(std::numeric_limits<double>::max_digits10);
        for (const auto& object : world.Objects())
        {
            const ObjectTransform& t = object->GetTransform();
            const auto& mesh = object->Instance().mesh;
            if (!mesh || mesh->assetPath.empty() || !std::isfinite(object->health) || object->health < 0.0f)
            {
                return SDL_SetError("Cannot save an invalid world object");
            }
            out << static_cast<unsigned>(object->Type()) << ' ' << std::quoted(mesh->assetPath)
                << ' ' << t.position.x << ' ' << t.position.y << ' ' << t.position.z
                << ' ' << t.rotation.x << ' ' << t.rotation.y << ' ' << t.rotation.z
                << ' ' << t.scale.x << ' ' << t.scale.y << ' ' << t.scale.z
                << ' ' << object->health << ' ' << object->invincible;
            if (const auto* vehicle = dynamic_cast<const Vehicle*>(object.get()))
            {
                if (!std::isfinite(vehicle->acceleration))
                {
                    return SDL_SetError("Cannot save a non-finite vehicle acceleration");
                }
                out << ' ' << vehicle->acceleration;
            }
            out << '\n';
        }
        return out.good() || SDL_SetError("Could not serialize world objects");
    }
}
