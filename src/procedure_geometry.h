#ifndef PROCEDURE_GEOMETRY_H
#define PROCEDURE_GEOMETRY_H

#include <vector>
#include <glm/glm.hpp>
#include "bone_geometry.h"

class LineMesh;

void create_floor(std::vector<glm::vec4>& floor_vertices, std::vector<glm::uvec3>& floor_faces);

void create_bones(std::vector<glm::vec4>& bone_vertices, std::vector<glm::uvec2>& bone_faces, Mesh &m);
// FIXME: Add functions to generate the bone mesh.

#endif
