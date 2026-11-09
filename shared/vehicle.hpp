#pragma once

#include "bscobject.hpp"

class Vehicle : public BSCObject
{
public:
    ObjectType Type() const override { return ObjectType::Vehicle; }
    float acceleration = 0.0f; // Meters per second squared.
};
