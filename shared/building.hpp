#pragma once

#include "bscobject.hpp"

class Building : public BSCObject
{
public:
    ObjectType Type() const override { return ObjectType::Building; }
    std::unique_ptr<BSCObject> Clone() const override { return std::make_unique<Building>(*this); }
};
