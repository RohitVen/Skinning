#ifndef BONE_GEOMETRY_H
#define BONE_GEOMETRY_H

#include <ostream>
#include <vector>
#include <map>
#include <limits>
#include <glm/glm.hpp>
#include <mmdadapter.h>
#include <iostream>
#include <cmath>

struct BoundingBox {
	BoundingBox()
		: min(glm::vec3(-std::numeric_limits<float>::max())),
		max(glm::vec3(std::numeric_limits<float>::max())) {}
	glm::vec3 min;
	glm::vec3 max;
};

struct Joint {
	// FIXME: Implement your Joint data structure.
	// Note: PMD represents weights on joints, but you need weights on
	//       bones to calculate the actual animation.
	Joint()
	{
		jointID = 0;
		parentID = 0;
		jointOffset = glm::vec3(0,0,0);
	}

	~Joint()
	{}

	void setValues(int id, int pid, glm::vec3 off)
	{
		jointID = id;
		parentID = pid;
		jointOffset = off;
	}

	int getID()
	{
		return jointID;
	}

	int getPID()
	{
		return parentID;
	}

	glm::vec3 getOffset()
	{
		return jointOffset;
	}


public:
	int jointID;
	int parentID;
	glm::vec3 jointOffset;
	// Joint children[];
	glm::mat4 coords;
	glm::mat4 rot;
	//Create and array of children
	//Store coords
};

struct LeDoot { //Bone data struct

	LeDoot()
	{}

	~LeDoot()
	{}

	void setValues(int i, int j)
	{
		src = i;
		dest = j;
	}

public:
	int src;
	int dest;
	//Should only have 2 ints



};

struct Skeleton {
	// FIXME: create skeleton and bone data structures
	Skeleton()
	{}

	~Skeleton()
	{}

	void buildJointStructure(MMDReader &mr)
	{
		int id = 0;
		glm::vec3 offset = {0,0,0};
		int parent = 0;
		numJoints = 0;
		while(mr.getJoint(id, offset, parent))
		{	
			Joint * j = new Joint();
			j->setValues(id, parent, offset);
			joints[id] = j;
			id++;
			if(parent != -1)
				numJoints++;
		}
	}

	void buildBoneStructure(MMDReader &mr)
	{
		Joint * j = new Joint();
		LeDoot b = LeDoot();
		LeDoot bones[numJoints];
		std::cout<<"\nNumJoints: "<<numJoints;

		for(int i = 0; i < numJoints; i++)
		{
			j = joints.at(i);
			int curr_id = j->getID();
			int curr_pid = j->getPID();
			if(curr_pid != -1)
			{
				std::cout<<"\nFound valid bone. Adding it "<<i;
				b.setValues(curr_pid, curr_id);
				bones[i] = b;
			}
			std::cout<<"\nAdded a bone! "<< i;
		}




		// for(int id = 0; id < numJoints; id++)
		// {
		// 	j = joints.at(id);
		// 	int curr_id = j->getID();
		// 	int curr_pid = j->getPID();
		// 	glm::vec3 curr_off = j->getOffset();
		// 	if(curr_pid != -1)
		// 	{
		// 		Joint* p = joints[curr_pid];
		// 		int par_id = p->getID();
		// 		int par_pid = p->getPID();
		// 		glm::vec3 par_off = p->getOffset();
		// 		tangent = par_off;
		// 		tangent = glm::normalize(tangent); //Tangent vector

		// 		glm::vec3 v = findSmallestComp(tangent);		
		// 		normal = glm::cross(tangent, v);
		// 		double magTV = normal[0]*normal[0] + normal[1]*normal[1] + normal[2]*normal[2];
		// 		magTV = sqrt(magTV);
		// 		normal = glm::vec3(normal[0]/magTV, normal[1]/magTV, normal[2]/magTV);
		// 		normal = glm::normalize(normal); //Normal vector

		// 		binormal = glm::cross(tangent, normal);
		// 		binormal = glm::normalize(binormal); //Binormal vector

		// 		std::cout<<"\n Joint num: "<<id;
		// 		std::cout<<"\n offset  vec: "<<curr_off[0]<<" "<<curr_off[1]<<" "<<curr_off[2];
		// 		std::cout<<"\n parent  vec: "<<par_off[0]<<" "<<par_off[1]<<" "<<par_off[2];
		// 		std::cout<<"\n v       vec: "<<v[0]<<" "<<v[1]<<" "<<v[2];
		// 		std::cout<<"\n tangent vec: "<<tangent[0]<<" "<<tangent[1]<<" "<<tangent[2];
		// 		std::cout<<"\n normal  vec: "<<normal[0]<<" "<<normal[1]<<" "<<normal[2];
		// 		std::cout<<"\n binrmal vec: "<<binormal[0]<<" "<<binormal[1]<<" "<<binormal[2];
		// 		std::cout<<"\n\n";

		// 		double magP = par_off[0]*par_off[0] + par_off[1]*par_off[1] + par_off[2]*par_off[2];
		// 		magP = sqrt(magP);
		// 		glm::vec4 tang = glm::vec4{tangent, 0};
		// 		glm::vec4 norm = glm::vec4{normal, 0};
		// 		glm::vec4 bi = glm::vec4{binormal, 0};
		// 		glm::vec4 temp = glm::vec4{magP,0,0,1};
		// 		glm::mat4 coords = glm::mat4{tang, norm, bi, temp};
		// 		glm::mat3 rot{tangent, normal, binormal};

		// 	}
		// }
	}

	glm::vec3 findSmallestComp(glm::vec3 &v)
	{
		double absX = fabs(v[0]);
		double absY = fabs(v[1]);
		double absZ = fabs(v[2]);
		if(absX < absY)
		{
			if(absX < absZ)
				return glm::vec3{1,0,0};
			else
				return glm::vec3{0,0,1};
		}
		else
		{
			if(absY < absZ)
				return glm::vec3{0,1,0};
			else
				return glm::vec3{0,0,1};
		}
	}

public:
	int numJoints;
	std::map<int, Joint *> joints;
	LeDoot bones[];
	glm::vec3 tangent; //x dir
	glm::vec3 normal; //z dir
	glm::vec3 binormal; //y dir

};

struct Mesh {
	Mesh();
	~Mesh();
	std::vector<glm::vec4> vertices;
	std::vector<glm::vec4> animated_vertices;
	std::vector<glm::uvec3> faces;
	std::vector<glm::vec4> vertex_normals;
	std::vector<glm::vec4> face_normals;
	std::vector<glm::vec2> uv_coordinates;
	std::vector<Material> materials;
	BoundingBox bounds;
	Skeleton skeleton;

	void loadpmd(const std::string& fn);
	void updateAnimation();
	int getNumberOfBones() const 
	{ 
		return numBones;
	}
	glm::vec3 getCenter() const { return 0.5f * glm::vec3(bounds.min + bounds.max); }
private:
	void computeBounds();
	void computeNormals();

public:
	int numBones = 0;
};

#endif
