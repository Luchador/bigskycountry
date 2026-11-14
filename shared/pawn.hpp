#pragma once

#include "bscobject.hpp"

class GameWorld;
class PlayerController;
using PawnHandle = std::uint64_t;

enum class PawnInputProfile { OnFoot, Driving };
enum class PawnCameraMode { FirstPerson, Chase };

struct PawnInput
{
    double forward = 0, right = 0, yaw = 0;
    bool sprint = false, jump = false, handbrake = false;
};

struct PawnCameraSettings
{
    PawnCameraMode mode = PawnCameraMode::FirstPerson;
    Vector3 pivot{0, 1.65, 0};
    double distance = 0;
};

class Pawn : public BSCObject
{
public:
    virtual PawnInputProfile InputProfile() const { return PawnInputProfile::OnFoot; }
    virtual PawnCameraSettings CameraSettings() const { return {}; }
    virtual void Simulate(double seconds, GameWorld& world) {}
    virtual bool BlocksSphere(Vector3 center, double radius) const { return false; }
    std::unique_ptr<BSCObject> Clone() const override { return std::make_unique<Pawn>(*this); }
    PawnHandle Handle() const { return handle; }
    bool IsPossessed() const { return controller != nullptr; }
    virtual void Teleport(const ObjectTransform& transform);
    ObjectTransform InterpolatedTransform(double fraction) const;
    const MeshInstance& RenderInstance(double fraction) const;
    Vector3 Forward() const;
    const PawnInput& Commands() const { return input; }

private:
    friend class GameWorld;
    friend class PlayerController;
    PawnHandle handle = 0;
    const PlayerController* controller = nullptr;
    PawnInput input{};
    ObjectTransform previous;
    mutable MeshInstance rendered;
};
