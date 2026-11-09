#include "mesh.hpp"

#include <cmath>

bool MeshTransform::Valid() const
{
    const auto finite = [](Vector3 v)
    {
        return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
    };
    const double determinant = Determinant();
    return finite(origin) && finite(x) && finite(y) && finite(z) &&
           std::isfinite(determinant) && determinant != 0.0;
}
