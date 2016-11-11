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
#include <climits>

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
	glm::vec4 check;
	std::vector<int> children;
	glm::mat4 trans;
	glm::mat3 rot;
	double length;
};

struct Bone { //Bone data struct

	Bone()
	{}

	~Bone()
	{}

	void setValues(int i, int j)
	{
		src = i;
		dest = j;
	}

public:
	int src;
	int dest;
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
		Joint j = Joint();
		while(mr.getJoint(id, offset, parent))
		{	
			j.setValues(id, parent, offset);
			joints.push_back(j);
			if(parent != -1)
			{
				joints[parent].children.push_back(id);
				numJoints++;
			}
			id++;
		}
	}

	void buildBoneStructure(MMDReader &mr)
	{
		Joint j = Joint();
		Bone b = Bone();
		numBones = 0;
		for(int i = 0; i < numJoints; i++)
		{
			j = joints[i];
			int curr_id = j.getID();
			int curr_pid = j.getPID();
			if(curr_pid != -1)
			{
				b.setValues(curr_pid, curr_id);
				bones.push_back(b);
				numBones++;
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

	void appendVertices(int s)
	{

		Joint j = joints[s];
		int par = j.parentID;

		if(j.children.size() <= 0 || s == numJoints - 1)
		{
			return;
		}

		if(s < numJoints && j.children.size() > 0)
		{
			for(int k = 0; k < j.children.size(); k++)
			{
				joints[j.children[k]].jointOffset.x += j.jointOffset.x;
				joints[j.children[k]].jointOffset.y += j.jointOffset.y;
				joints[j.children[k]].jointOffset.z += j.jointOffset.z;
			}
			// std::cout<<"\nnumchild should be 0. So I'm here!";
			glm::vec3 par_off = joints[j.getPID()].getOffset();
			tangent = par_off;
			tangent = glm::normalize(tangent); //Tangent vector

			glm::vec3 v = findSmallestComp(tangent);		
			normal = glm::cross(tangent, v);
			double magTV = normal[0]*normal[0] + normal[1]*normal[1] + normal[2]*normal[2];
			magTV = sqrt(magTV);
			normal = glm::vec3(normal[0]/magTV, normal[1]/magTV, normal[2]/magTV);
			normal = glm::normalize(normal); //Normal vector


			binormal = glm::cross(tangent, normal);
			binormal = glm::normalize(binormal); //Binormal vector

			double magP = par_off[0]*par_off[0] + par_off[1]*par_off[1] + par_off[2]*par_off[2];
			magP = sqrt(magP);

			glm::mat4 trans = glm::mat4{glm::vec4{0,0,0,0}, glm::vec4{0,0,0,0}, glm::vec4{0,0,0,0}, glm::vec4{j.jointOffset, 0}};
			glm::mat3 rot{normal, tangent, binormal};
			j.trans = trans;
			j.rot = rot;
			j.length = magP;
			std::cout<<"\ntangent: "<<tangent[0]<<" "<<tangent[1]<<" "<<tangent[2];
			std::cout<<"\nnormal : "<<normal[0]<<" "<<normal[1]<<" "<<normal[2];
			std::cout<<"\nbinrmal: "<<binormal[0]<<" "<<binormal[1]<<" "<<binormal[2];
			appendVertices(s+1);
		}		
	}

public:
	int numJoints;
	int numBones;
	std::vector<Joint> joints;
	std::vector<Bone> bones;
	glm::vec3 tangent; //x dir
	glm::vec3 normal; //y dir
	glm::vec3 binormal; //z dir

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
