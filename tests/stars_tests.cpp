#include "stars.hpp"
#include "camera_view.hpp"
#include "sky_location.hpp"
#include "sun.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace
{
void Check(bool ok, const char* message)
{
    if (!ok) throw std::runtime_error(message);
}
Vector3 Equatorial(double raHours, double decDegrees)
{
    const double ra = Math::DegreesToRadians(15 * raHours), dec = Math::DegreesToRadians(decDegrees);
    return {std::cos(dec)*std::cos(ra), std::cos(dec)*std::sin(ra), std::sin(dec)};
}
const StarSky::Star& Find(Vector3 p)
{
    const auto stars = StarSky::Catalog();
    return *std::max_element(stars.begin(), stars.end(), [&](const auto& a, const auto& b) {
        return Math::Dot(p, {a.x,a.y,a.z}) < Math::Dot(p, {b.x,b.y,b.z});
    });
}
void CatalogAndSky()
{
    const auto stars = StarSky::Catalog();
    Check(stars.size() == 8920, "Unexpected HYG magnitude-limited catalog size");
    for (const auto& s : stars)
    {
        Check(std::isfinite(s.magnitude) && s.magnitude <= 6.5f && s.magnitude > -2, "Magnitude filter/Sun exclusion failed");
        Check(std::abs(Math::Length({s.x,s.y,s.z})-1) < 1e-6, "Invalid celestial direction");
        for (float c : {s.r,s.g,s.b,s.brightness}) Check(std::isfinite(c) && c >= 0 && c <= 1, "Invalid color/brightness");
    }
    const auto& sirius = Find(Equatorial(6.7525, -16.7161));
    const auto& betelgeuse = Find(Equatorial(5.9195, 7.4071));
    Check(std::abs(sirius.magnitude + 1.44f) < .03f, "Sirius missing/mispositioned");
    Check(std::abs(betelgeuse.magnitude - .45f) < .05f, "Betelgeuse missing/mispositioned");
    Check(sirius.b / sirius.r > betelgeuse.b / betelgeuse.r, "Star colors lost warm/cool distinction");
    Check(sirius.brightness > betelgeuse.brightness, "Magnitude brightness reversed");
    for (int hour = 0; hour < 24; ++hour)
    {
        const auto frame = StarSky::At(60 * hour);
        const auto polaris = frame.ToWorld(Equatorial(2.5303, 89.2641));
        const double elevation = std::asin(polaris.y) * 180 / std::numbers::pi;
        Check(std::abs(elevation - SkyLocation::latitudeDegrees) < 1, "Polaris altitude incorrect");
        Check(polaris.z > .7 && std::abs(polaris.x) < .02, "Polaris does not point north");
    }
    const auto vega = Equatorial(18.6156, 38.7837);
    Check(StarSky::At(0).ToWorld(vega).y > .85, "Summer midnight sky orientation incorrect");
    Check(Math::Length(StarSky::At(1439.999).ToWorld(vega) - StarSky::At(0).ToWorld(vega)) < 1e-5,
          "Stars jump when game time wraps at midnight");
    Check(StarSky::HorizonFade(-.01) == 0 && StarSky::HorizonFade(0) == 0, "Stars show below horizon");
    Check(StarSky::DaylightFade(Sun::Direction(720).y) == 0, "Stars show at noon");
    Check(StarSky::DaylightFade(Sun::Direction(0).y) == 1, "Stars not fully visible at midnight");
    const auto atElevation = [](double degrees) { return StarSky::DaylightFade(std::sin(Math::DegreesToRadians(degrees))); };
    Check(atElevation(-6) == 1 && atElevation(5) == 0, "Fade thresholds incorrect");
    Check(atElevation(1) > 0 && atElevation(1) < 1, "No fade just above the horizon");
    float previous = 1;
    for (int i = -60; i <= 50; ++i)
    {
        const float fade = atElevation(i / 10.0);
        Check(fade <= previous, "Sunrise fade is not monotonic");
        previous = fade;
    }
}

std::vector<Uint8> Capture(SDL_Renderer* renderer, Stars& stars, const CameraView& camera,
                           double time, int width, int height)
{
    Check(SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255) && SDL_RenderClear(renderer), SDL_GetError());
    Check(stars.Draw(renderer, camera, time, Sun::Direction(time).y, {0,0,width,height}), SDL_GetError());
    auto* raw = SDL_RenderReadPixels(renderer, nullptr);
    Check(raw != nullptr, SDL_GetError());
    auto* image = SDL_ConvertSurface(raw, SDL_PIXELFORMAT_RGBA32);
    SDL_DestroySurface(raw);
    Check(image != nullptr, SDL_GetError());
    std::vector<Uint8> rgb;
    for (int y=0; y<image->h; ++y) for (int x=0; x<image->w; ++x)
    {
        const auto* p = static_cast<const Uint8*>(image->pixels) + y*image->pitch + x*4;
        rgb.insert(rgb.end(), p, p+3);
    }
    SDL_DestroySurface(image);
    return rgb;
}
void Render(bool gpu, const char* screenshot)
{
    Check(SDL_Init(SDL_INIT_VIDEO), SDL_GetError());
    SDL_Window* window = nullptr;
    SDL_Surface* surface = nullptr;
    SDL_Renderer* renderer = nullptr;
    if (gpu)
    {
        window = SDL_CreateWindow("Star rendering regression", 800, 600, SDL_WINDOW_HIDDEN);
        Check(window != nullptr, SDL_GetError());
        renderer = SDL_CreateRenderer(window, "gpu");
    }
    else
    {
        surface = SDL_CreateSurface(800, 600, SDL_PIXELFORMAT_RGBA32);
        Check(surface != nullptr, SDL_GetError());
        renderer = SDL_CreateSoftwareRenderer(surface);
    }
    Check(renderer != nullptr, SDL_GetError());
    {
        Stars stars;
        CameraView camera;
        camera.LookAt({}, {0,.5,1});
        const auto night = Capture(renderer, stars, camera, 0, 800, 600);
        const auto lit = [](const auto& data) { return std::count_if(data.begin(), data.end(), [](auto c) { return c > 0; }); };
        Check(lit(night) > 500, "Night sky has no visible stars");
        Check(Capture(renderer, stars, camera, 0, 800, 600) == night, "Cached star frame changed");
        camera.LookAt({10000,200,3000}, {10000,200.5,3001});
        Check(Capture(renderer, stars, camera, 0, 800, 600) == night, "Stars have camera translation parallax");
        Check(lit(Capture(renderer, stars, camera, 720, 800, 600)) == 0, "Daytime render contains stars");
        Check(Capture(renderer, stars, camera, 0, 800, 600) == night, "Stars failed to return after daytime");
        if (screenshot)
        {
            auto* image = SDL_RenderReadPixels(renderer, nullptr);
            Check(image && SDL_SaveBMP(image, screenshot), SDL_GetError());
            SDL_DestroySurface(image);
        }
        camera.LookAt({}, {0,-1,.01});
        Check(lit(Capture(renderer, stars, camera, 0, 800, 600)) == 0, "Stars rendered beneath the horizon");
        camera.LookAt({}, {1,.5,0});
        Check(Capture(renderer, stars, camera, 0, 800, 600) != night, "Camera rotation did not change star field");
        // Exercise offscreen target reallocation / projection dimensions.
        const auto resized = Capture(renderer, stars, camera, 0, 400, 300);
        Check(lit(resized) > 100, "Resized star viewport is empty");
    }
    SDL_DestroyRenderer(renderer);
    SDL_DestroySurface(surface);
    SDL_DestroyWindow(window);
    SDL_Quit();
}
}

int main(int argc, char** argv)
{
    try
    {
        CatalogAndSky();
        Render(argc > 1 && std::string_view(argv[1]) == "--gpu", argc > 2 ? argv[2] : nullptr);
        std::cout << "Star catalog, sky orientation, fades and rendering passed\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
