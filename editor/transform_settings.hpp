#pragma once

#include <SDL3/SDL_surface.h>
#include <array>

enum class GizmoSpace { World, Local };
enum class TransformIcon { World, Local, SnapOff, SnapOn };
inline constexpr std::array transformIcons{TransformIcon::World, TransformIcon::Local,
                                           TransformIcon::SnapOff, TransformIcon::SnapOn};

class TransformSettings
{
public:
    GizmoSpace space = GizmoSpace::World;
    bool snapEnabled = false;

    void ToggleSpace() { space = space == GizmoSpace::World ? GizmoSpace::Local : GizmoSpace::World; }
    TransformIcon SpaceIcon() const { return space == GizmoSpace::World ? TransformIcon::World : TransformIcon::Local; }
    TransformIcon SnapIcon() const { return snapEnabled ? TransformIcon::SnapOn : TransformIcon::SnapOff; }

    void ChangeStep(int direction);
    double StepMeters() const;
    const char* StepLabel() const;
    double SnapDistance(double meters) const;
    bool operator==(const TransformSettings&) const = default;

private:
    int step = 2; // Start at 1 m.
};

// The caller owns the returned surface.
SDL_Surface* LoadTransformIcon(TransformIcon icon);
