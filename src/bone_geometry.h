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
		check = off;
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
	glm::vec3 check;
	std::vector<int> children;
	glm::mat4 trans;
	glm::mat3 basis;
	glm::mat4 rot;
	glm::mat4 coords;
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
			}
			numJoints++;
			id++;
		}
		joints.push_back(j);
	}

	void buildBoneStructure(MMDReader &mr)
	{
		Joint j = Joint();
		Bone b = Bone();
		numBones = 0;
		for(int i = 0; i < joints.size(); i++)
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
		int par = joints[s].parentID;

		if(s >= joints.size())
		{
			//Base case
			return;
		}

		if(s < joints.size())
		{
			double magP = joints[s].check[0]*joints[s].check[0] + joints[s].check[1]*joints[s].check[1] + joints[s].check[2]*joints[s].check[2];
			magP = sqrt(magP);
			joints[s].length = magP; //Length of joint

			for(int k = 0; k < j.children.size(); k++)
			{
				//Test offset for use against transform matrix
				joints[j.children[k]].jointOffset.x += joints[s].jointOffset.x;
				joints[j.children[k]].jointOffset.y += joints[s].jointOffset.y;
				joints[j.children[k]].jointOffset.z += joints[s].jointOffset.z;
			}
			if(s != 0)
			{
				// std::cout<<"\npoffst: "<<joints[par].jointOffset.x<<" "<<joints[par].jointOffset.y<<" "<<joints[par].jointOffset.z;
				// std::cout<<"\noffset: "<<joints[s].check.x<<" "<<joints[s].check.y<<" "<<joints[s].check.z;

				//Finding Transform Matrix
				glm::mat4 coords = joints[par].coords;
				glm::vec4 trans_coords = glm::inverse(coords) * glm::vec4(joints[s].check,1);
				trans_coords[3] = 1;

				joints[s].trans = glm::mat4(glm::vec4(1,0,0,0), glm::vec4(0,1,0,0), glm::vec4(0,0,1,0), trans_coords); //Translation Matrix

				glm::vec4 start = coords * joints[s].trans * glm::vec4(0,0,0,1); //FINAL START COORDS

				glm::mat4 accum = joints[par].coords * joints[s].trans;
				glm::vec4 rot_coords = glm::inverse(accum) * glm::vec4(joints[s].check + joints[par].check, 1);
				rot_coords[3] = 1;

				//Finding Rotation Matrix
				glm::vec3 par_off = joints[par].getOffset();
				tangent = glm::vec3(rot_coords[0]/magP, rot_coords[1]/magP, rot_coords[2]/magP); //Tangent vector

				glm::vec3 v = findSmallestComp(tangent);		
				normal = glm::cross(tangent, v);
				double magTV = normal[0]*normal[0] + normal[1]*normal[1] + normal[2]*normal[2];
				magTV = sqrt(magTV);

				normal = glm::vec3(normal[0]/magTV, normal[1]/magTV, normal[2]/magTV); //Normal vector

				binormal = glm::cross(tangent, normal); //Binormal vector

				glm::mat4 rot{glm::vec4(normal,0), glm::vec4(binormal,0), glm::vec4(tangent,0), glm::vec4(0,0,0,1)};
				joints[s].rot = rot; //Rotation Matrix

				joints[s].coords = joints[s].trans * joints[s].rot;

				glm::vec4 end = accum * joints[s].rot * glm::vec4(0,0,magP,1); //FINAL END COORDS

				// std::cout<<"\nstart : "<<start.x<<" "<<start.y<<" "<<start.z;
				// std::cout<<"\nfinal : "<<joints[s].jointOffset.x<<" "<<joints[s].jointOffset.y<<" "<<joints[s].jointOffset.z;
				// std::cout<<"\n\n";
				joints[s].check = glm::vec3(end);

			} 
			else
			{
				//Root joint
				joints[s].trans = glm::mat4{glm::vec4{1,0,0,0}, glm::vec4{0,1,0,0}, glm::vec4{0,0,1,0}, glm::vec4{j.jointOffset, 1}}; //Translation Matrix

				glm::mat4 rot{glm::vec4{1,0,0,0}, glm::vec4{0,1,0,0}, glm::vec4{0,0,1,0}, glm::vec4{0,0,0,1}};
				joints[s].rot = rot; //Rotation Matrix

				glm::mat4 coords = joints[s].trans * joints[s].rot;
				joints[s].coords = coords; //Coords Matrix

				//Basis Matrix
				glm::mat3 basis = glm::mat3{binormal, normal, tangent};
				joints[s].basis = basis;
			}
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
		return skeleton.bones.size();
	}
	glm::vec3 getCenter() const { return 0.5f * glm::vec3(bounds.min + bounds.max); }
private:
	void computeBounds();
	void computeNormals();

public:
	int numBones = 0;
};

#endif
