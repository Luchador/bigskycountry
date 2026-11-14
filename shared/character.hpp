#pragma once

#include "pawn.hpp"

class Character : public Pawn
{
public:
    ObjectType Type() const override { return ObjectType::Character; }
    std::unique_ptr<BSCObject> Clone() const override { return std::make_unique<Character>(*this); }
    void Simulate(double seconds, GameWorld& world) override;
    bool BlocksSphere(Vector3 center, double radius) const override;
    void SetSeated(bool value);
    bool IsSeated() const { return seated; }
    bool IsGrounded() const { return grounded; }
    Vector3 Velocity() const { return velocity; }

private:
    Vector3 velocity{};
    bool grounded = false, seated = false;
};
