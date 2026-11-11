#pragma once

#include <string>

enum class AssetRoot { Project, Cooked };

namespace AssetPaths
{
    // UTF-8 directories with a trailing slash, independent of the working directory.
    const std::string& Root(AssetRoot location);
}
