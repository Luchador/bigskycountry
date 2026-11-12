#pragma once

#include "bscobject.hpp"

class Vehicle : public BSCObject
{
public:
    ObjectType Type() const override { return ObjectType::Vehicle; }
    std::unique_ptr<BSCObject> Clone() const override { return std::make_unique<Vehicle>(*this); }
    float acceleration = 0.0f; // Meters per second squared.
};
