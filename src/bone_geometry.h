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
		// std::cout<<"\nREACHED HERE\n";
		Joint * j = new Joint();
		std::cout<<"\nNumJoints: "<<numJoints;
		for(int id = 0; id < numJoints; id++)
		{
			j = joints.at(id);
			int curr_id = j->getID();
			int curr_pid = j->getPID();
			glm::vec3 curr_off = j->getOffset();
			if(curr_pid != -1)
			{
				Joint* p = joints[curr_pid];
				int par_id = p->getID();
				int par_pid = p->getPID();
				glm::vec3 par_off = p->getOffset();
				tangent = par_off;
				tangent = glm::normalize(tangent); //Tangent vector

				glm::vec3 v = findSmallestComp(tangent);
				double magT = tangent[0]*tangent[0] + tangent[1]*tangent[1] + tangent[2]*tangent[2];
				magT = sqrt(magT);
				double magV = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
				magV = sqrt(magV);
				normal = glm::cross(tangent, v);
				normal = glm::normalize(normal); //Normal vector

				binormal = glm::cross(tangent, v);
				binormal = glm::normalize(binormal); //Binormal vector
			}
		}
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
	glm::vec3 tangent;
	glm::vec3 normal;
	glm::vec3 binormal;

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
