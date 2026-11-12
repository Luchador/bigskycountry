#pragma once

#include "mesh.hpp"

// Stable type IDs in the OBJS world section.
enum class ObjectType : unsigned
{
    Object = 0,
    Building = 1,
    Vehicle = 2,
    Prop = 3,
    Character = 4,
    Marker = 5
};

// One placed actor. Instances share an immutable model but have independent state.
class BSCObject : public MeshPlacement
{
public:
    virtual ~BSCObject() = default;
    virtual ObjectType Type() const { return ObjectType::Object; }
    // Override alongside Type() so editor copies preserve the concrete object and its properties.
    virtual std::unique_ptr<BSCObject> Clone() const { return std::make_unique<BSCObject>(*this); }
    virtual void Tick(float deltaSeconds);
    virtual void TakeDamage(float damage);

    bool invincible = false;
    float health = 100.0f;
};
