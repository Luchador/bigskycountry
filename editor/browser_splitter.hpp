#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

class BrowserSplitter
{
public:
    // Keep the chosen width across resizes, scaled with the platform's UI size.
    // Temporarily clamp it in small windows without losing that preference.
    int Width(int available, int defaultWidth, int minViewport, std::optional<int> dragWidth = {})
    {
        available = std::max(0, available);
        const int low = std::min(defaultWidth, available / 2);
        const int high = std::max(available - minViewport, available / 2);
        const int preferred = scale ? static_cast<int>(std::lround(*scale * defaultWidth)) : defaultWidth;
        const int current = std::clamp(preferred, low, high);
        if (!dragWidth || defaultWidth <= 0) return current;
        const int next = std::clamp(*dragWidth, low, high);
        if (next != current) scale = static_cast<double>(next) / defaultWidth;
        return next;
    }

private:
    std::optional<double> scale;
};
