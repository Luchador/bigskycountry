#pragma once

#include <cstdint>
#include <iosfwd>
#include <string>

class WorldDocument;

namespace ObjectIO
{
    inline constexpr std::uint32_t currentVersion = 1;
    bool Read(std::string payload, WorldDocument& world);
    bool Write(std::ostream& out, const WorldDocument& world);
}
