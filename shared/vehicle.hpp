#pragma once

#include "pawn.hpp"

class Vehicle : public Pawn
{
public:
    ObjectType Type() const override { return ObjectType::Vehicle; }
    std::unique_ptr<BSCObject> Clone() const override { return std::make_unique<Vehicle>(*this); }
    PawnInputProfile InputProfile() const override { return PawnInputProfile::Driving; }
    PawnCameraSettings CameraSettings() const override { return {PawnCameraMode::Chase, {0, 1.1, 0}, 6.0}; }
    virtual Vector3 SeatPosition() const { return {0, 0.7, 0}; }
    float acceleration = 0.0f; // Meters per second squared.
};
