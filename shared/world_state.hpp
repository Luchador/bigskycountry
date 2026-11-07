#pragma once

#include <cstdint>

class WorldState
{
    public:
        // Minutes since midnight, including the fractional part, in [0, 1440).
        double GetTimeOfDayMinutes() const;
        void SetTimeOfDayMinutes(int minutes);
        void AdvanceTime(std::uint64_t elapsedRealNanoseconds);

    private:
        std::uint64_t realNanosecondsIntoDay = 0;
};

// Each executable owns one world state.
extern WorldState g_WorldState;
