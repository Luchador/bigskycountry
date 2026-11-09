#pragma once

#include "bscobject.hpp"

class Character : public BSCObject
{
public:
    ObjectType Type() const override { return ObjectType::Character; }
};
