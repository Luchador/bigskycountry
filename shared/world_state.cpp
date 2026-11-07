#include "world_state.hpp"

namespace
{
    constexpr int minutesPerDay = 24 * 60;
    constexpr std::uint64_t realNanosecondsPerGameMinute = 2'000'000'000;
    constexpr std::uint64_t realNanosecondsPerGameDay = minutesPerDay * realNanosecondsPerGameMinute;
}

WorldState g_WorldState;

double WorldState::GetTimeOfDayMinutes() const
{
    return static_cast<double>(realNanosecondsIntoDay) / realNanosecondsPerGameMinute;
}

void WorldState::SetTimeOfDayMinutes(int minutes)
{
    const int minuteOfDay = ((minutes % minutesPerDay) + minutesPerDay) % minutesPerDay;
    realNanosecondsIntoDay = minuteOfDay * realNanosecondsPerGameMinute;
}

void WorldState::AdvanceTime(std::uint64_t elapsedRealNanoseconds)
{
    // Discard whole days before adding so even a very large elapsed interval fits.
    const std::uint64_t remainder = elapsedRealNanoseconds % realNanosecondsPerGameDay;
    realNanosecondsIntoDay = (realNanosecondsIntoDay + remainder) % realNanosecondsPerGameDay;
}
