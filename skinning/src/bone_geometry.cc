#include <iostream>
#include "config.h"
#include "bone_geometry.h"
#include "texture_to_render.h"
#include <fstream>
#include <queue>
#include <stdexcept>
#include <glm/gtx/io.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/string_cast.hpp>

using namespace std;

/*
 * For debugging purpose.
 */
template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& v) {
	size_t count = std::min(v.size(), static_cast<size_t>(10));
	for (size_t i = 0; i < count; ++i) os << i << " " << v[i] << "\n";
	os << "size = " << v.size() << "\n";
	return os;
}

std::ostream& operator<<(std::ostream& os, const BoundingBox& bounds)
{
	os << "min = " << bounds.min << " max = " << bounds.max;
	return os;
}



const glm::vec3* Skeleton::collectJointTrans() const
{
	return cache.trans.data();
}

const glm::fquat* Skeleton::collectJointRot() const
{
	return cache.rot.data();
}

// FIXME: Implement bone animation.

void Skeleton::refreshCache(Configuration* target)
{
	if (target == nullptr)
		target = &cache;
	target->rot.resize(joints.size());
	target->trans.resize(joints.size());
	for (size_t i = 0; i < joints.size(); i++) {
		target->rot[i] = joints[i].orientation;
		target->trans[i] = joints[i].position;
	}
}


Mesh::Mesh()
{
}

Mesh::~Mesh()
{
}

void Mesh::loadPmd(const std::string& fn)
{
	// vector<glm::vec4> vertices;
	// vector<glm::uvec3> faces;
	// vector<glm::vec4> vertex_normals;
	// vector<glm::vec2> uv_coordinates;

	MMDReader mr;
	mr.open(fn);
	mr.getMesh(vertices, faces, vertex_normals, uv_coordinates);
	computeBounds();
	mr.getMaterial(materials);

	// FIXME: load skeleton and blend weights from PMD file,
	//        initialize std::vectors for the vertex attributes,
	//        also initialize the skeleton as needed
	// for(int i=0; i<vertices.size(); i++) {
	// 	cout << glm::to_string(vertices[i]) << endl;
	// }

	glm::vec3 w_coords;
	int parent;
	int count = 0;
	while (mr.getJoint(count, w_coords, parent)) {
		Joint j (count, w_coords, parent);
		skeleton.joints.push_back(j);
		
		++count;
	}
	
	// Build the joint data in the skeleton
	for (int i = 1; i < skeleton.joints.size(); ++i) {
		// cout << "Creating joint: " << i << endl;
		Joint cur = skeleton.joints[i];
		Joint* parent = &skeleton.joints[cur.parent_index];
		parent->children.emplace_back(i);
		// cout << "Parent " << parent->joint_index << "'s Children: " << parent->children;
	}
	cout << "NUM JOINTS: " << skeleton.joints.size() << endl;

	// Build the bones
	// 1) loop through joints and build individual bones
	for(int i=0; i<skeleton.joints.size(); i++) {
		Joint* from = &(skeleton.joints[i]);
		// cout << "SIZE OF CHILDREN " << from.children.size() << endl;
		for (int j = 0; j < from->children.size(); ++j) {
			int child = from->children[j];
			Joint* to = &(skeleton.joints[child]);
			Bone b (from, to);
			skeleton.bones.push_back(b);
		}
	}
	
	cout << "NUM BONES: " << skeleton.bones.size() << endl;
	// Set up the T and R matricies. Find the children of each bone and add it to its children
	for(int i=0; i<skeleton.bones.size(); i++) {
		Bone* bone = &skeleton.bones[i];
		bone->length = glm::length(bone->to->position - bone->from->position);
		bone->rot = glm::mat4();
		bone->trans = glm::mat4();
		cout << "From: " << bone->from->joint_index << " -> " << bone->to->joint_index << endl;
		glm::vec4 t = glm::vec4(glm::normalize(bone->to->position - bone->from->position), 0);
		glm::vec3 v = glm::vec3(t);
		int index = 0;
		for(int j=0; j<v.length(); j++) {
			if(v[j] < v[index]) {
				index = j;
			}
		}
		for(int j=0; j<v.length(); j++) {
			if(j == index)
				v[j] = 1;
			else
				v[j] = 0;
		}
		glm::vec4 n = glm::vec4(glm::cross(glm::vec3(t), v) / glm::length(glm::cross(glm::vec3(t), v)), 0);
		glm::vec4 b = glm::vec4(glm::cross(glm::vec3(t), glm::vec3(n)), 0);
		glm::mat4 rot;
		rot[0] = t;
		rot[1] = n;
		rot[2] = b;
		rot[3] = glm::vec4(0,0,0,1);
		bone->rot = glm::transpose(rot);
		bone->trans = glm::mat4();
		// if base, use world coordinates as T
		if (bone->from->joint_index == 0) {
			glm::vec3 tra = bone->from->init_position;
			bone->trans[0][0] = tra[0];
			bone->trans[1][1] = tra[1];
			bone->trans[2][2] = tra[2];
			bone->trans[3][3] = 1;
			cout << "Translation Vector: " << glm::to_string(tra) << endl;
			
		}
		else {
			bone->trans[0][0] = bone->length;
			bone->trans[1][1] = 0;
			bone->trans[2][2] = 0;
			bone->trans[3][3] = 1;
		}
		cout << "Translation Matrix: " << glm::to_string(bone->trans) << endl;
		// Find the children for the current bone and add it to its children vector
		for(int k=i+1; k<skeleton.bones.size(); k++) {
			Bone* child = &skeleton.bones[k];
			if(child->from->joint_index == bone->to->joint_index) {
				bone->children.emplace_back(child);
			}
		}
	}

	for(int i=0; i<5; i++) {
		Bone* b = &skeleton.bones[i];
		cout << "CHILDREN SIZE: " << b->children.size() << endl;
		cout << "TRANS: " << glm::to_string(b->trans) << endl;
		cout << "rot: " << glm::to_string(b->rot) << endl;
	}
}

int Mesh::getNumberOfBones() const
{
	return skeleton.joints.size();
}

void Mesh::computeBounds()
{
	bounds.min = glm::vec3(std::numeric_limits<float>::max());
	bounds.max = glm::vec3(-std::numeric_limits<float>::max());
	for (const auto& vert : vertices) {
		bounds.min = glm::min(glm::vec3(vert), bounds.min);
		bounds.max = glm::max(glm::vec3(vert), bounds.max);
	}
}

void Mesh::updateAnimation(float t)
{
	skeleton.refreshCache(&currentQ_);
	// FIXME: Support Animation Here
}

const Configuration*
Mesh::getCurrentQ() const
{
	return &currentQ_;
}

