#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

class OutlinerSplitter
{
public:
    // The available height excludes the bar. Keep the user's split proportional on resize.
    int Height(int available, int defaultLimit, int minOutliner, int minProperties,
               std::optional<int> dragHeight = {})
    {
        const int third = available / 3;
        const int low = std::min(minOutliner, third);
        const int high = std::max(available - minProperties, third);
        const int preferred = fraction ? static_cast<int>(std::lround(*fraction * available)) :
                                         std::min(defaultLimit, third);
        const int current = std::clamp(preferred, low, high);
        if (!dragHeight || available == 0) return current;

        const int next = std::clamp(*dragHeight, low, high);
        if (next != current) fraction = static_cast<double>(next) / available;
        return next;
    }

private:
    std::optional<double> fraction;
};
