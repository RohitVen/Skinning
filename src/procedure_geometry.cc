#include "procedure_geometry.h"
#include "bone_geometry.h"
#include "config.h"
#include <iostream>
#include <cmath>

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
	for(int i = 0; i < m.skeleton.bones.size(); i++)
	{
		b = m.skeleton.bones[i];
		bone_vertices.push_back(glm::vec4(b.start,1));
		bone_vertices.push_back(glm::vec4(b.end,1));

	}

	for(int i = 0; i < 2*m.skeleton.bones.size(); i = i+2)
	{
		bone_faces.push_back(glm::uvec2(i,i+1));
	}
}

void create_cylinder(std::vector<glm::vec4>& cyl_vertices, std::vector<glm::uvec2>& cyl_faces, Mesh &m, int bone)
{
	double radius = kCylinderRadius;
	double toRad = M_PI/180;
	double deg = 0;
	int ind = 0;
	int num = 0;
	Bone b = m.skeleton.bones[bone];
	double len = b.length;
	while(deg < 360)
	{
		double rad = deg * toRad;
		cyl_vertices.push_back(b.coords*(glm::vec4(0, radius*cos(rad), radius*sin(rad), 1)));
		deg += 45;
		rad = deg * toRad;
		cyl_vertices.push_back(b.coords*(glm::vec4(0, radius*cos(rad), radius*sin(rad), 1)));
	}
	deg = 0;
	while(deg < 360)
	{
		double rad = deg * toRad;
		cyl_vertices.push_back(b.coords*(glm::vec4(len, radius*cos(rad), radius*sin(rad), 1)));
		deg += 45;
		rad = deg * toRad;
		cyl_vertices.push_back(b.coords*(glm::vec4(len, radius*cos(rad), radius*sin(rad), 1)));
	}
	deg = 0;
	while(deg < 360)
	{
		double rad = deg * toRad;
		cyl_vertices.push_back(b.coords*(glm::vec4(0, radius*cos(rad), radius*sin(rad), 1)));
		cyl_vertices.push_back(b.coords*(glm::vec4(len, radius*cos(rad), radius*sin(rad), 1)));
		deg += 45;
	}
	while(ind < 48)
	{
		cyl_faces.push_back(glm::uvec2(ind, ind+1));
		ind += 2;	
	}
}

// FIXME: create cylinders and lines for the bones
// Hints: Generate a lattice in [-0.5, 0, 0] x [0.5, 1, 0] We wrap this
// around in the vertex shader to produce a very smooth cylinder.  We only
// need to send a small number of points.  Controlling the grid size gives a
// nice wireframe.
