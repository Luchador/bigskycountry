#pragma once

#include "bscobject.hpp"

class Prop : public BSCObject
{
public:
    ObjectType Type() const override { return ObjectType::Prop; }
};
