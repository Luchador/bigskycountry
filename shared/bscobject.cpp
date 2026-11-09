#include "bscobject.hpp"

#include <algorithm>
#include <cmath>

void BSCObject::Tick(float)
{
}

void BSCObject::TakeDamage(float damage)
{
    if (!invincible && std::isfinite(damage) && damage > 0.0f)
    {
        health = std::max(0.0f, health - damage);
    }
}
