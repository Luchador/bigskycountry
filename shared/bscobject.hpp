/**
 * BSCObjects are world actors with a position, rotation, and scale. They are used for most things that are not roads or terrain
 * e.g. buildings, vehicles, signs, people, animals.
 */

#pragma once

#include "math.hpp"

class BSCObject
{
    public:
        virtual void Tick(float deltaTime);
        virtual void TakeDamage(float damage);
        bool isLoaded;
        bool invincible;
        float health = 100.0f;
};