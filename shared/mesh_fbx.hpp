#pragma once
#include <cstddef>
struct Mesh;
bool ImportFBX(const void* bytes, std::size_t size, Mesh& mesh);
