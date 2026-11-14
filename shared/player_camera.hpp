#pragma once

#include "camera_view.hpp"
#include "pawn.hpp"

class WorldCollision;

class PlayerCamera
{
public:
    void Possess(const Pawn& pawn);
    void Look(double x, double y);
    void Update(const Pawn& pawn, double fraction, double seconds, const WorldCollision& collision);
    double Yaw() const { return yaw; }
    const CameraView& View() const { return view; }

private:
    CameraView view, transitionStart;
    double yaw = 0, pitch = 0, lastPawnYaw = 0, blend = 1;
    bool initialized = false;
};
