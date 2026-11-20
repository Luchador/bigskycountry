#pragma once

// The sun and stars share the existing repeating August 1 sky at this location.
namespace SkyLocation
{
    inline constexpr double latitudeDegrees = 38.0;
    inline constexpr double longitudeDegrees = -95.0;
    inline constexpr double utcOffsetHours = -5.0; // Central Daylight Time.
    inline constexpr int dayOfYear = 213; // August 1, non-leap year.
    inline constexpr double julianDate = 2452122.5; // 2001-08-01, 00:00 UTC.
}
