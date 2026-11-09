#pragma once

#include "mesh.hpp"

// Stable type IDs in the OBJS world section.
enum class ObjectType : unsigned
{
    Object = 0,
    Building = 1,
    Vehicle = 2,
    Prop = 3,
    Character = 4
};

// One placed actor. Instances share an immutable model but have independent state.
class BSCObject : public MeshPlacement
{
public:
    virtual ~BSCObject() = default;
    virtual ObjectType Type() const { return ObjectType::Object; }
    virtual void Tick(float deltaSeconds);
    virtual void TakeDamage(float damage);

    bool invincible = false;
    float health = 100.0f;
};
