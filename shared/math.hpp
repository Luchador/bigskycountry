#pragma once

#include <numbers>

class Math
{
public:
    Math() = delete;

    struct Vector3
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;

        constexpr Vector3 operator+(const Vector3& other) const
        {
            return { x + other.x, y + other.y, z + other.z };
        }

        constexpr Vector3 operator-(const Vector3& other) const
        {
            return { x - other.x, y - other.y, z - other.z };
        }

        constexpr Vector3 operator*(double scale) const
        {
            return { x * scale, y * scale, z * scale };
        }
    };

    static double Dot(const Vector3& a, const Vector3& b);
    static Vector3 Cross(const Vector3& a, const Vector3& b);
    static double Length(const Vector3& vector);

    // A zero vector has no direction and normalizes to zero.
    static Vector3 Normalize(const Vector3& vector);

    static constexpr double DegreesToRadians(double degrees)
    {
        return degrees * (std::numbers::pi / 180.0);
    }
};

using Vector3 = Math::Vector3;
