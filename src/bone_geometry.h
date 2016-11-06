#ifndef BONE_GEOMETRY_H
#define BONE_GEOMETRY_H

#include <ostream>
#include <vector>
#include <map>
#include <limits>
#include <glm/glm.hpp>
#include <mmdadapter.h>
#include <iostream>

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
	Joint(int id, int pid, glm::vec3 off)
	{
		jointID = id;
		parentID = pid;
		jointOffset = off;
	}

	~Joint()
	{}

	bool buildJoint(MMDReader &mr, int idVal, glm::vec3& offset, int& parent)
	{
		jointID = idVal;
		if(mr.getJoint(jointID, offset, parent))
		{
			std::cout<<"id, parent: "<<jointID<<" "<<parent<<"\n";
			parentID = parent;
			jointOffset[0] = offset[0];
			jointOffset[1] = offset[1];
			jointOffset[2] = offset[2];
			return true;
		}
		return false;
	}


public:
	int jointID;
	int parentID;
	int numBones;
	glm::vec3 jointOffset;
};

// struct LeDoot { //Bone class

// }


struct Skeleton {
	// FIXME: create skeleton and bone data structures
	Skeleton()
	{}

	~Skeleton()
	{}

	void buildBoneStructure(MMDReader &mr, int num)
	{
		int id = 0;
		glm::vec3 offset = {0,0,0};
		int parent = 0;
		numBones = num;
		// joints = new Joint[numBones];
		while(joints->buildJoint(mr, id, offset, parent))
		{
			id++;
		}
	}

public:
	int numBones;
	int *joint; //Index is joint ID, value is parent ID
	Joint *joints;


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
