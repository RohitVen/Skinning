#include "procedure_geometry.h"
#include "bone_geometry.h"
#include "config.h"
#include <iostream>

void create_floor(std::vector<glm::vec4>& floor_vertices, std::vector<glm::uvec3>& floor_faces)
{
	floor_vertices.push_back(glm::vec4(kFloorXMin, kFloorY, kFloorZMax, 1.0f));
	floor_vertices.push_back(glm::vec4(kFloorXMax, kFloorY, kFloorZMax, 1.0f));
	floor_vertices.push_back(glm::vec4(kFloorXMax, kFloorY, kFloorZMin, 1.0f));
	floor_vertices.push_back(glm::vec4(kFloorXMin, kFloorY, kFloorZMin, 1.0f));
	floor_faces.push_back(glm::uvec3(0, 1, 2));
	floor_faces.push_back(glm::uvec3(2, 3, 0));
}

void create_bones(std::vector<glm::vec4>& bone_vertices, std::vector<glm::uvec2>& bone_faces, Mesh &m)
{
	int src;
	int dest;
	Bone b;
	Joint j;
	for(int i = 0; i < m.skeleton.numBones; i++)
	{
		b = m.skeleton.bones[i];
		src = b.src;
		dest = b.dest;
		j = m.skeleton.joints[src];

		bone_vertices.push_back(glm::vec4(j.jointOffset,1.0f));
		// std::cout<<"\nsrc: "<<j.jointOffset[0]<<" "<<j.jointOffset[1]<<" "<<j.jointOffset[2];
		// std::cout<<"\nsrc check: "<<j.check[0]<<" "<<j.check[1]<<" "<<j.check[2];
		// std::cout<<"\n";
		j = m.skeleton.joints[dest];
		bone_vertices.push_back(glm::vec4(j.jointOffset,1.0f));
		// std::cout<<"\ndest: "<<j.jointOffset[0]<<" "<<j.jointOffset[1]<<" "<<j.jointOffset[2];
		// std::cout<<"\ndest check: "<<j.check[0]<<" "<<j.check[1]<<" "<<j.check[2];
		// std::cout<<"\n\n";


	}

	for(int i = 0; i < 2*m.skeleton.numBones; i = i+2)
	{
		bone_faces.push_back(glm::uvec2(i,i+1));
	}
}

// FIXME: create cylinders and lines for the bones
// Hints: Generate a lattice in [-0.5, 0, 0] x [0.5, 1, 0] We wrap this
// around in the vertex shader to produce a very smooth cylinder.  We only
// need to send a small number of points.  Controlling the grid size gives a
// nice wireframe.
