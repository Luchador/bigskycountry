#pragma once

namespace AssetBuild
{
    // Run the configured, incremental BigSkyAssets target before world cooking.
    // Failure leaves the previous world package untouched; details go to the build log.
    bool Run();
}
