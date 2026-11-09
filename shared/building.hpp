#pragma once

#include "bscobject.hpp"

class Building : public BSCObject
{
public:
    ObjectType Type() const override { return ObjectType::Building; }
};
