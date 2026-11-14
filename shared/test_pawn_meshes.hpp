#pragma once

#include "mesh.hpp"

// Small procedural stand-ins make the first gameplay test self-contained.
namespace TestPawnMeshes
{
    std::shared_ptr<const Mesh> Car();
    std::shared_ptr<const Mesh> Character();
    std::shared_ptr<const Mesh> Ground();
}
