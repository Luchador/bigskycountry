#pragma once

#include <cstdint>
#include <iosfwd>
#include <string>

class WorldDocument;

namespace ObjectIO
{
    inline constexpr std::uint32_t currentVersion = 3;
    bool Read(std::string payload, std::uint32_t version, WorldDocument& world);
    bool Write(std::ostream& out, const WorldDocument& world);
}
