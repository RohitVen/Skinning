#ifndef BONE_GEOMETRY_H
#define BONE_GEOMETRY_H

#include <ostream>
#include <vector>
#include <map>
#include <limits>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
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
		end = off;
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
	glm::vec3 end;
	std::vector<int> children;
	glm::mat4 trans;
	glm::mat3 basis;
	glm::mat4 rot;
	glm::mat4 total_rot;
	glm::mat4 coords;
	glm::vec3 check;
	// glm::mat4 orientation;
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
	double length;
	glm::vec3 start;
	glm::vec3 end;
	glm::mat4 trans;
	glm::mat4 rot;
	glm::mat4 coords;
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
		if(absX <= absY)
		{
			if(absX <= absZ)
				return glm::vec3{1,0,0};
			else
				return glm::vec3{0,0,1};
		}
		else
		{
			if(absY <= absZ)
				return glm::vec3{0,1,0};
			else
				return glm::vec3{0,0,1};
}
	}

	void appendVertices(int s)
	{

		if(s >= joints.size())
		{
			//Base case
			return;
		}

		if(s < joints.size())
		{
			Joint j = joints[s];
			int par = joints[s].parentID;

			double magP = glm::length(joints[s].end);
				joints[s].length = magP;

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
				bones[s-1].length = magP;
				glm::mat4 coords = joints[par].coords;
				glm::vec4 st = bones[par].coords * glm::vec4(0,0,0,1);
				glm::vec4 en = bones[par].coords * glm::vec4(0,0,bones[par-1].length,1);
				glm::vec4 trans_coords = glm::inverse(coords) * glm::vec4(joints[par].jointOffset,1);
				trans_coords[3] = 1;

				joints[s].trans = glm::mat4(glm::vec4(1,0,0,0), glm::vec4(0,1,0,0), glm::vec4(0,0,1,0), trans_coords); //Translation Matrix

				glm::vec4 start = joints[par].coords * joints[s].trans * glm::vec4(0,0,0,1); //FINAL START COORDS

				glm::mat4 accum = joints[par].coords * joints[s].trans;
				glm::vec4 rot_coords = glm::inverse(accum) * glm::vec4(joints[s].jointOffset, 1);
				glm::vec4 ans = glm::vec4(joints[s].jointOffset - joints[par].jointOffset, 1); 
				rot_coords[3] = 1;

				//Finding Rotation Matrix
				// glm::vec3 par_off = joints[par].getOffset();
				glm::mat4 iden = glm::mat4(1);
				iden[3][3] = 0;
				glm::mat4 temp = joints[par].rot * iden;

				glm::vec4 tang = glm::transpose(temp) * glm::vec4(joints[s].check, 0); 
				tang[3] = 1;
				// glm::vec3 tangent = glm::vec3(tang);
				glm::vec3 tangent = glm::vec3(rot_coords);
				tangent = glm::normalize(tangent); //Tangent vector

				glm::vec3 v = findSmallestComp(tangent);		
				normal = glm::cross(tangent, v);
				normal = glm::normalize(normal);


				binormal = glm::cross(tangent, normal); //Binormal vector
				// binormal = glm::normalize(binormal);

				glm::mat4 rot{glm::vec4(tangent,0), glm::vec4(binormal,0), glm::vec4(normal,0), glm::vec4(0,0,0,1)};
				joints[s].rot = rot; //Rotation Matrix

				// joints[s].total_rot = joints[par].total_rot * rot;

				// joints[s].orientation = glm::inverse(joints[s].rot);

				joints[s].coords = accum * joints[s].rot;

				glm::vec4 end = accum * joints[s].rot * glm::vec4(magP,0,0,1); //FINAL END COORDS

				// std::cout<<"\nstart : "<<start.x<<" "<<start.y<<" "<<start.z;
				// std::cout<<"\nfinal : "<<joints[s].jointOffset.x<<" "<<joints[s].jointOffset.y<<" "<<joints[s].jointOffset.z;
				// std::cout<<"\n\n";
				joints[s].end = glm::vec3(end);

				bones[s-1].start = glm::vec3(start);
				bones[s-1].end = glm::vec3(end);
				bones[s-1].trans = joints[s].trans;
				bones[s-1].rot = joints[s].rot;
				bones[s-1].coords = joints[s].coords;

				//ERROR CHECKING
				if(s==3)
				{
				// 	std::cout<<"\nsrc and dest: "<<par<<" "<<s;
					std::cout<<"\nbone start  : "<<bones[par-1].start.x<<" "<<bones[par-1].start.y<<" "<<bones[par-1].start.z;
					std::cout<<"\nbone end    : "<<bones[s-1].end.x<<" "<<bones[s-1].end.y<<" "<<bones[s-1].end.z;
				// std::cout<<"\ncurrent joint and its parent: "<<s<<" "<<par;
					std::cout<<"\npar_offset  : "<<joints[par].jointOffset.x<<" "<<joints[par].jointOffset.y<<" "<<joints[par].jointOffset.z;
				// 	// std::cout<<"\nOriginal off: "<<joints[s].org.x<<" "<<joints[s].org.y<<" "<<joints[s].org.z;
					std::cout<<"\nchild_offset: "<<joints[s].jointOffset.x<<" "<<joints[s].jointOffset.y<<" "<<joints[s].jointOffset.z;
					std::cout<<"\npar_org_off : "<<joints[par].check.x<<" "<<joints[par].check.y<<" "<<joints[par].check.z;
					std::cout<<"\nchld_org_of : "<<joints[s].check.x<<" "<<joints[s].check.y<<" "<<joints[s].check.z;
					std::cout<<"\nTang        : "<<tang.x<<" "<<tang.y<<" "<<tang.z;

				// 	// std::cout<<"\nMagT: "<<magT;
				// 	std::cout<<"\nStart vector: "<<start.x<<" "<<start.y<<" "<<start.z;
				// 	std::cout<<"\nEnd vector  : "<<end.x<<" "<<end.y<<" "<<end.z;
				// 	std::cout<<"\nTrans coords: "<<trans_coords.x<<" "<<trans_coords.y<<" "<<trans_coords.z;
					std::cout<<"\nRot coords  : "<<rot_coords.x<<" "<<rot_coords.y<<" "<<rot_coords.z;
					std::cout<<"\nAns         : "<<ans.x<<" "<<ans.y<<" "<<ans.z;
					std::cout<<"\njoints[s].ck: "<<joints[s].check.x<<" "<<joints[s].check.y<<" "<<joints[s].check.z;

				// 	std::cout<<"\nv vector    : "<<v.x<<" "<<v.y<<" "<<v.z;
					std::cout<<"\nNormal vectr: "<<normal.x<<" "<<normal.y<<" "<<normal.z;
					std::cout<<"\nBinormal    : "<<binormal.x<<" "<<binormal.y<<" "<<binormal.z;
					std::cout<<"\nTangent     : "<<tangent.x<<" "<<tangent.y<<" "<<tangent.z;
					std::cout<<"\npar bone s and d: "<<bones[par-1].src<<" "<<bones[par-1].dest;
					// std::cout<<"\ns: "<<st.x<<" "<<st.y<<" "<<st.z;
					// std::cout<<"\ne: "<<en.x<<" "<<en.y<<" "<<en.z;

					std::cout<<"\n\nParent bone coords\n";
					for(int i=0;i<4;i++)
					{
						for(int j=0;j<4;j++)
						{
							std::cout<<joints[par].coords[i][j]<<" ";
						}
						std::cout<<"\n";
					}
					std::cout<<"\n\nParent bone trans\n";
					for(int i=0;i<4;i++)
					{
						for(int j=0;j<4;j++)
						{
							std::cout<<joints[par].trans[i][j]<<" ";
						}
						std::cout<<"\n";
					}
					std::cout<<"\n\nParent bone rot\n";
					for(int i=0;i<4;i++)
					{
						for(int j=0;j<4;j++)
						{
							std::cout<<joints[par].rot[i][j]<<" ";
						}
						std::cout<<"\n";
					}


					// std::cout<<"\nans         : "<<ans.x<<" "<<ans.y<<" "<<ans.z<<"\n";
					std::cout<<"\n\nTrans\n";
					for(int i=0;i<4;i++)
					{
						for(int j=0;j<4;j++)
						{
							std::cout<<joints[s].trans[i][j]<<" ";
						}
						std::cout<<"\n";
					}

					// std::cout<<"\nPar Trans\n";
					// for(int i=0;i<4;i++)
					// {
					// 	for(int j=0;j<4;j++)
					// 	{
					// 		std::cout<<joints[par].trans[i][j]<<" ";
					// 	}
					// 	std::cout<<"\n";
					// }

					std::cout<<"\nrot\n";
					for(int i=0;i<4;i++)
					{
						for(int j=0;j<4;j++)
						{
							std::cout<<joints[s].rot[i][j]<<" ";
						}
						std::cout<<"\n";
					}
					std::cout<<"\n";

					// std::cout<<"\nPar rot\n";
					// for(int i=0;i<4;i++)
					// {
					// 	for(int j=0;j<4;j++)
					// 	{
					// 		std::cout<<joints[par].rot[i][j]<<" ";
					// 	}
					// 	std::cout<<"\n";
					// }
					// std::cout<<"\n";

					// std::cout<<"\ntotal rot\n";
					// for(int i=0;i<4;i++)
					// {
					// 	for(int j=0;j<4;j++)
					// 	{
					// 		std::cout<<joints[s].total_rot[i][j]<<" ";
					// 	}
					// 	std::cout<<"\n";
					// }
					// std::cout<<"\n";

					// std::cout<<"\ncoords\n";
					// for(int i=0;i<4;i++)
					// {
					// 	for(int j=0;j<4;j++)
					// 	{
					// 		std::cout<<joints[s].coords[i][j]<<" ";
					// 	}
					// 	std::cout<<"\n";
					// }
					// std::cout<<"\n\n";

					// glm::mat4 test = glm::inverse(joints[par].coords);

					// std::cout<<"\npar coords inverse\n";
					// for(int i=0;i<4;i++)
					// {
					// 	for(int j=0;j<4;j++)
					// 	{
					// 		std::cout<<test[i][j]<<" ";
					// 	}
					// 	std::cout<<"\n";
					// }
					// std::cout<<"\n\n";

					// std::cout<<"\naccum coords\n";
					// accum = glm::inverse(accum);
					// for(int i=0;i<4;i++)
					// {
					// 	for(int j=0;j<4;j++)
					// 	{
					// 		std::cout<<accum[i][j]<<" ";
					// 	}
					// 	std::cout<<"\n";
					// }
					// std::cout<<"\n\n";
				}

			} 
			else
			{
				//Root joint
				joints[s].trans = glm::mat4{glm::vec4{1,0,0,0}, glm::vec4{0,1,0,0}, glm::vec4{0,0,1,0}, glm::vec4{0,0,0,1}}; //Translation Matrix

				glm::mat4 rot{glm::vec4{1,0,0,0}, glm::vec4{0,1,0,0}, glm::vec4{0,0,1,0}, glm::vec4{0,0,0,1}};
				joints[s].rot = rot; //Rotation Matrix

				joints[s].total_rot = rot;

				glm::mat4 coords = joints[s].trans * joints[s].rot;
				joints[s].coords = coords; //Coords Matrix

				// //Basis Matrix
				// glm::mat3 basis = glm::mat3{binormal, normal, tangent};
				// joints[s].basis = basis;
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
