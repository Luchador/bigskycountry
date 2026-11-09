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

struct ObjectTransform
{
    Vector3 position{};
    Vector3 rotation{};       // Radians about X, Y, Z, applied in that order.
    Vector3 scale{1, 1, 1};   // Along the object's local axes.

    MeshTransform ToMeshTransform() const;
};

// One placed actor. Instances share an immutable model but have independent state.
class BSCObject
{
public:
    virtual ~BSCObject() = default;
    virtual ObjectType Type() const { return ObjectType::Object; }
    virtual void Tick(float deltaSeconds);
    virtual void TakeDamage(float damage);

    const ObjectTransform& GetTransform() const { return transform; }
    bool SetTransform(const ObjectTransform& value);
    void SetMesh(std::shared_ptr<const Mesh> mesh);
    bool IsLoaded() const { return instance.mesh != nullptr; }
    const MeshInstance& Instance() const { return instance; }

    bool invincible = false;
    float health = 100.0f;

private:
    ObjectTransform transform;
    MeshInstance instance; // Updated on transform edits. Rendering and picking share this matrix.
};
