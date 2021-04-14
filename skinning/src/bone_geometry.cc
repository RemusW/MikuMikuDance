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
		// cout << w_coords << endl;
		++count;
	}
	
	// Build the joint data in the skeleton
	cout << "Building joint data" << endl;
	for (size_t i = 1; i < skeleton.joints.size(); ++i) {
		// cout << "Creating joint: " << i << endl;
		Joint cur = skeleton.joints[i];
		Joint* parent = &skeleton.joints[cur.parent_index];
		parent->children.emplace_back(i);
		// cout << "Parent " << parent->joint_index << "'s Children: " << parent->children;
	}
	cout << "Num joints created: " << skeleton.joints.size() << endl;
	
	// Build the bones
	// Create root bone -1
	cout << "Building bones" << endl;
	Joint origin;
	Bone root (&origin, &skeleton.joints[0]);
	root.trans = glm::mat4(1);
	root.rot = glm::mat4(1);
	skeleton.bones.push_back(root);
	int id = 1;
	// loop through joints and build individual bones
	for(size_t i=0; i<skeleton.joints.size(); i++) {
		Joint* from = &(skeleton.joints[i]);
		// cout << "SIZE OF CHILDREN " << from.children.size() << endl;
		for (int j = 0; j < from->children.size(); ++j) {
			int child = from->children[j];
			Joint* to = &(skeleton.joints[child]);
			Bone b (from, to, id++);
			skeleton.bones.push_back(b);
		}
	}
	cout << "Num bones created: " << skeleton.bones.size() << endl;
	
	// Set up the T and R matricies. Find the children of each bone and add it to its children
	for(size_t i=0; i<skeleton.bones.size(); i++) {
		Bone* bone = &skeleton.bones[i];
		// cout << "Translation Matrix: " << glm::to_string(bone->trans) << endl;
		// Find the children for the current bone and add it to its children vector
		for(int k=i+1; k<skeleton.bones.size(); k++) {
			Bone* child = &skeleton.bones[k];
			if(child->from->joint_index == bone->to->joint_index) {
				bone->children.emplace_back(child);
			}
		}
	}
	Bone* r = &skeleton.bones[0];
	glm::mat4 M = glm::mat4(1.0f);
	recurseBoneTree(&skeleton.bones[0], M, true);

	// vector<Joint> dummy_data;
	// vector<Bone> dummy_bones;
	// dummy_data.push_back(Joint(0, glm::vec3(0, 0, 0), -1));
	// dummy_data.push_back(Joint(1, glm::vec3(0, 2, 0), 0));
	// dummy_data.push_back(Joint(2, glm::vec3(1, 2, 0), 1));
	// dummy_data.push_back(Joint(3, glm::vec3(1, 1, 0), 2));
	// dummy_data.push_back(Joint(4, glm::vec3(3, 1, 0), 3));
	// Joint dummy_origin;
	// Bone dummy_root (&dummy_origin, &dummy_data[0]);
	// dummy_root.trans = glm::mat4(1);
	// dummy_root.rot = glm::mat4(1);
	// dummy_bones.push_back(dummy_root);
	// for (int i = 0; i < dummy_data.size() - 1; ++i) {
	// 	Bone b(&dummy_data[i], &dummy_data[i + 1]);
	// 	dummy_bones.push_back(b);
	// }
	// for (int i = 0; i < dummy_bones.size() - 1; ++i) {
	// 	dummy_bones[i].children.emplace_back(&dummy_bones[i+1]);

	// }
	// recurseBoneTree(&dummy_bones[0], M, 0);
}

// Paremeters: current bone, M of parents
// find cur bone's from and to position in parent's coords
// calculate cur bone's rot and trans w.r.t. parent
// upda
void Mesh::recurseBoneTree(Bone* bone, glm::mat4 M, float parent_length) {
	if(bone->from->joint_index != -1) {
		// intialize basic values
		bone->length = glm::length(bone->to->init_position - bone->from->init_position);
		bone->rot = glm::mat4(1.0f);
		bone->trans = glm::mat4(1.0f);
		bone->init_M = M;

		// calculate positions in parent coords
		glm::mat4 invM = glm::inverse(M);
		glm::vec4 from_pc = invM * glm::vec4(bone->from->position, 1);
		glm::vec4 to_pc = invM * glm::vec4(bone->to->position, 1);
		
		// cout << "From: " << bone->from->joint_index << " -> " << bone->to->joint_index << endl;
		// cout << "	from: " << from_pc << endl;
		// cout << "	to:   " << to_pc << endl;
		// cout << "	M: 		" << M << endl;
		// cout << "	invM: 	" << invM << endl;
		
		// calculate the rotation
		glm::vec3 t = glm::normalize(glm::vec3(to_pc) - glm::vec3(from_pc));
		glm::vec3 v = glm::vec3(t);
		int index = 0;
		for(int j=0; j<v.length(); j++) {
			if(abs(v[j]) < abs(v[index])) {
				index = j;
			}
		}
		for(int j=0; j<v.length(); j++) {
			if(j == index)
				v[j] = 1;
			else
				v[j] = 0;
		}
		glm::vec3 n = glm::cross(t, v);
		glm::vec3 b = glm::cross(t, glm::vec3(n));
		glm::mat4 rot;
		rot[0] = glm::vec4(t, 0);
		rot[1] = glm::vec4(n, 0);
		rot[2] = glm::vec4(b, 0);
		rot[3] = glm::vec4(0,0,0,1);
		bone->rot = rot;
		// bone->rot = glm::transpose(rot);


		// calc Translation
		bone->trans = glm::mat4(1);
		if(bone->from->joint_index == 0) {
			bone->trans[0][3] = glm::length(bone->from->position - glm::vec3(0,0,0));
			bone->trans = glm::transpose(bone->trans);
		}
		else {
			bone->trans[0][3] = parent_length;
			bone->trans = glm::transpose(bone->trans);
		}
		parent_length = bone->length;
		// cout << "	rotation: 	" << bone->rot << endl;
		// cout << "	translate: 	" << bone->trans << endl;

		// cout << "	tangent: 	" << t << endl;
		// cout << "	v: 		 " << v << endl;
		// cout << "	normal: 	" << n << endl;
		// cout << "	binormal: 	" << b << endl;
		
		M = M * bone->trans * bone->rot;
		// cout << "		source:    " << (M * glm::vec4(0,0,0,1) ) << endl;
		// cout << "		world pos: " << bone->from->position << endl;
		// cout << "		dest: 	   " << (M * glm::vec4(bone->length,0,0,1)) << endl;
		// cout << "		world pos: " << bone->to->position << endl;
		
		
		// M = bone->rot * bone->trans * M ;
		
	}
	for (size_t i=0; i<bone->children.size(); ++i) {
		recurseBoneTree(bone->children[i], M, bone->length);
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

