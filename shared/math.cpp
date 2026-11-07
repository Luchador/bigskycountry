#include "math.hpp"

#include <cmath>

double Math::Dot(const Vector3& a, const Vector3& b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vector3 Math::Cross(const Vector3& a, const Vector3& b)
{
    return
    {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

double Math::Length(const Vector3& vector)
{
    return std::hypot(vector.x, vector.y, vector.z);
}

Vector3 Math::Normalize(const Vector3& vector)
{
    const double length = Length(vector);
    if (length == 0.0)
    {
        return {};
    }

    return { vector.x / length, vector.y / length, vector.z / length };
}
