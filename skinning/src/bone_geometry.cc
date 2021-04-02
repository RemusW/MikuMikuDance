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
			cout << skeleton.bones.size() << endl;
		}
		// Bone b (i, )
	}
	cout << skeleton.bones.size() << endl;
	for(int i=0; i<skeleton.bones.size(); i++) {
		Bone b = skeleton.bones[i];
		cout << "From: " << b.from->joint_index << " -> " << b.to->joint_index << endl;
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

