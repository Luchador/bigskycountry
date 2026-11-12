#pragma once

#include "bscobject.hpp"

class Prop : public BSCObject
{
public:
    ObjectType Type() const override { return ObjectType::Prop; }
    std::unique_ptr<BSCObject> Clone() const override { return std::make_unique<Prop>(*this); }
};
