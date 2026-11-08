#pragma once

#include "road.hpp"

#include <array>
#include <cstdint>
#include <string>

class WorldDocument
{
public:
    std::vector<Road> roads;

    bool Open(const char* documentPath);
    bool Save() const;

private:
    struct Section
    {
        std::array<char, 4> tag;
        std::uint32_t version;
        std::string payload;
    };

    std::string path;
    std::vector<Section> otherSections; // Copied verbatim when this build edits a world with newer data types.
};
