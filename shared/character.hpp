#pragma once

#include "bscobject.hpp"

class Character : public BSCObject
{
public:
    ObjectType Type() const override { return ObjectType::Character; }
    std::unique_ptr<BSCObject> Clone() const override { return std::make_unique<Character>(*this); }
};
