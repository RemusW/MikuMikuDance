#include "gui.h"
#include "config.h"
#include <jpegio.h>
#include "bone_geometry.h"
#include <iostream>
#include <algorithm>
#include <debuggl.h>
#include <glm/gtc/matrix_access.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/intersect.hpp>

using namespace std;

namespace {
	// FIXME: Implement a function that performs proper
	//        ray-cylinder intersection detection
	// TIPS: The implement is provided by the ray-tracer starter code.
}

GUI::GUI(GLFWwindow* window, int view_width, int view_height, int preview_height)
	:window_(window), preview_height_(preview_height)
{
	glfwSetWindowUserPointer(window_, this);
	glfwSetKeyCallback(window_, KeyCallback);
	glfwSetCursorPosCallback(window_, MousePosCallback);
	glfwSetMouseButtonCallback(window_, MouseButtonCallback);
	glfwSetScrollCallback(window_, MouseScrollCallback);

	glfwGetWindowSize(window_, &window_width_, &window_height_);
	if (view_width < 0 || view_height < 0) {
		view_width_ = window_width_;
		view_height_ = window_height_;
	} else {
		view_width_ = view_width;
		view_height_ = view_height;
	}
	float aspect_ = static_cast<float>(view_width_) / view_height_;
	projection_matrix_ = glm::perspective((float)(kFov * (M_PI / 180.0f)), aspect_, kNear, kFar);
}

GUI::~GUI()
{
}

void GUI::assignMesh(Mesh* mesh)
{
	mesh_ = mesh;
	center_ = mesh_->getCenter();
}

void GUI::keyCallback(int key, int scancode, int action, int mods)
{
#if 0
	if (action != 2)
		std::cerr << "Key: " << key << " action: " << action << std::endl;
#endif
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
		glfwSetWindowShouldClose(window_, GL_TRUE);
		return ;
	}
	if (key == GLFW_KEY_J && action == GLFW_RELEASE) {
		//FIXME save out a screenshot using SaveJPEG
	}
	if (key == GLFW_KEY_S && (mods & GLFW_MOD_CONTROL)) {
		if (action == GLFW_RELEASE)
			mesh_->saveAnimationTo("animation.json");
		return ;
	}

	if (mods == 0 && captureWASDUPDOWN(key, action))
		return ;
	if (key == GLFW_KEY_LEFT || key == GLFW_KEY_RIGHT) {
		float roll_speed;
		if (key == GLFW_KEY_RIGHT)
			roll_speed = -roll_speed_;
		else
			roll_speed = roll_speed_;
		// FIXME: actually roll the bone here
	} else if (key == GLFW_KEY_C && action != GLFW_RELEASE) {
		fps_mode_ = !fps_mode_;
	} else if (key == GLFW_KEY_LEFT_BRACKET && action == GLFW_RELEASE) {
		current_bone_--;
		current_bone_ += mesh_->getNumberOfBones();
		current_bone_ %= mesh_->getNumberOfBones();
	} else if (key == GLFW_KEY_RIGHT_BRACKET && action == GLFW_RELEASE) {
		current_bone_++;
		current_bone_ += mesh_->getNumberOfBones();
		current_bone_ %= mesh_->getNumberOfBones();
	} else if (key == GLFW_KEY_T && action != GLFW_RELEASE) {
		transparent_ = !transparent_;
	}

	// FIXME: implement other controls here.
}

void GUI::mousePosCallback(double mouse_x, double mouse_y)
{
	last_x_ = current_x_;
	last_y_ = current_y_;
	current_x_ = mouse_x;
	current_y_ = window_height_ - mouse_y;
	float delta_x = current_x_ - last_x_;
	float delta_y = current_y_ - last_y_;
	if (sqrt(delta_x * delta_x + delta_y * delta_y) < 1e-15)
		return;
	if (mouse_x > view_width_)
		return ;
	glm::vec3 mouse_direction = glm::normalize(glm::vec3(delta_x, delta_y, 0.0f));
	glm::vec2 mouse_start = glm::vec2(last_x_, last_y_);
	glm::vec2 mouse_end = glm::vec2(current_x_, current_y_);
	glm::uvec4 viewport = glm::uvec4(0, 0, view_width_, view_height_);

	bool drag_camera = drag_state_ && current_button_ == GLFW_MOUSE_BUTTON_RIGHT;
	bool drag_bone = drag_state_ && current_button_ == GLFW_MOUSE_BUTTON_LEFT;

	if (drag_camera) {
		glm::vec3 axis = glm::normalize(
				orientation_ *
				glm::vec3(mouse_direction.y, -mouse_direction.x, 0.0f)
				);
		orientation_ =
			glm::mat3(glm::rotate(rotation_speed_, axis) * glm::mat4(orientation_));
		tangent_ = glm::column(orientation_, 0);
		up_ = glm::column(orientation_, 1);
		look_ = glm::column(orientation_, 2);
	} else if (drag_bone && current_bone_ != -1) {
		// FIXME: Handle bone rotation

		// glm::mat4 M_world = current_M_;
		// M_world[0][3] = 0;
		// M_world = glm::inverse(M_world);
		// glm::vec3 tvec = M_world[0];
		// // glm::vec3 axis = glm::cross(look_, tvec);
		// // glm::mat4 rotDef = glm::rotate(glm::mat4(1), 3.14f/180*rotation_speed_, look_);
		// M_world = glm::rotate(M_world, 3.14f/180*rotation_speed_, look_);

		// mesh_->skeleton.bones[current_bone_].rot = glm::inverse();
		return ;
	}

	// FIXME: highlight bones that have been moused over
	glm::vec3 world_coord = glm::unProject(glm::vec3(current_x_, current_y_, 0),
		view_matrix_ * model_matrix_, projection_matrix_,
		glm::vec4(0, 0, window_width_, window_height_));

	glm::vec4 ray = glm::vec4(glm::normalize(world_coord - eye_), 1);
	ray = glm::vec4(glm::vec3(0,0,0) - eye_, 1);
	
	// Joint fow(-2, eye_, -1);
	// Joint tow(-2, world_coord_VM, -1);
	// Bone line (&fow, &tow);
	// mesh_->skeleton.bones.emplace_back(line);
	// mesh_->skeleton.bones[0].children.emplace_back(&line);

	Bone* b = &mesh_->skeleton.bones[0];
	float id = -1;
	float t = 9999999;
	raycylinder_intersect(&mesh_->skeleton.bones[0], glm::mat4(1), ray, id, t);
	int index = -1;
	for(size_t i=0; i<mesh_->skeleton.bones.size(); ++i) {
		if(id == mesh_->skeleton.bones[i].id)
			index = i;
	}
	cout << id << " " << t << endl;
	cout << glm::to_string(world_coord) << endl;
	cout << glm::to_string(ray) << endl;
	current_bone_ = index;
}

void GUI::raycylinder_intersect(Bone* bone, glm::mat4 M_parent, glm::vec4 ray, float& id, float& t) {
	// not the root bone, check intersection
	if(bone->id != -1 && bone->from->parent_index == -1) {
		// transform ray into bone local
		glm::mat4 M = M_parent * bone->trans * bone->rot;
		glm::mat4 invM = glm::inverse(M);
		glm::vec4 local_ray = invM * ray;
		local_ray = glm::normalize(ray);

		// calculate t in y-z plane
		glm::vec4 o = invM * glm::vec4(eye_, 1);
		// cout << glm::to_string(ray) << endl;
		// cout << o << endl;
		float ox = o[1];
		float oy = o[2];
		float dx = local_ray[1];
		float dy = local_ray[2];
		float a = dx*dx + dy*dy;
		float b = 2*ox*dx + 2*oy*dy;
		float c = ox*ox + oy*oy - kCylinderRadius*kCylinderRadius;
		float discriminant = b*b-4*a*c;
		
		if (discriminant > 0) {
			// cout << "	a: " << a << endl;
			// cout << "	b: " << b << endl;
			// cout << "	c: " << c << endl;
			float positive = (-b + sqrt(discriminant)) / (2*a);
			float negative = (-b - sqrt(discriminant)) / (2*a);
			if (positive > 0 && negative > 0) {
				float temp_t;
				glm::vec3 pos1 = glm::vec3(o) + glm::vec3(local_ray)*positive;
				glm::vec3 pos2 = glm::vec3(o) + glm::vec3(local_ray)*negative;
				bool good = pos1[0] > 0 && pos1[0] < bone->length && pos2[0] > 0 && pos2[0] < bone->length;
				if(good) {
					temp_t = min(positive, negative);
					if(temp_t < t) {
						t = temp_t;
						id = bone->id;
						current_M_ = M;
						current_M_[0][3] = bone->length;
					}
				}
				// cout << "intersection with " << bone->id << " " << temp_t << endl;
				// cout << "	" << sqrt(discriminant) << " " << positive << " " << negative << " " << bone->id << endl;
				// cout << "	" << glm::to_string(o) << endl;
				// cout << "	" << glm::to_string(pos1) << endl;
				// cout << "	" << glm::to_string(pos2) << endl;
			}
		}
		
		M_parent = M;
	}
	
	for (size_t i=0; i<bone->children.size(); ++i) {
		raycylinder_intersect(bone->children[i], M_parent, ray, id, t);
	}
}

void GUI::mouseButtonCallback(int button, int action, int mods)
{
	if (current_x_ <= view_width_) {
		drag_state_ = (action == GLFW_PRESS);
		current_button_ = button;
		return ;
	}
	// FIXME: Key Frame Selection
}

void GUI::mouseScrollCallback(double dx, double dy)
{
	if (current_x_ < view_width_)
		return;
	// FIXME: Mouse Scrolling
}

void GUI::updateMatrices()
{
	// Compute our view, and projection matrices.
	if (fps_mode_)
		center_ = eye_ + camera_distance_ * look_;
	else
		eye_ = center_ - camera_distance_ * look_;

	view_matrix_ = glm::lookAt(eye_, center_, up_);
	light_position_ = glm::vec4(eye_, 1.0f);

	aspect_ = static_cast<float>(view_width_) / view_height_;
	projection_matrix_ =
		glm::perspective((float)(kFov * (M_PI / 180.0f)), aspect_, kNear, kFar);
	model_matrix_ = glm::mat4(1.0f);
}

MatrixPointers GUI::getMatrixPointers() const
{
	MatrixPointers ret;
	ret.projection = &projection_matrix_;
	ret.model= &model_matrix_;
	ret.view = &view_matrix_;
	return ret;
}

bool GUI::setCurrentBone(int i)
{
	if (i < 0 || i >= mesh_->getNumberOfBones())
		return false;
	current_bone_ = i;
	return true;
}

float GUI::getCurrentPlayTime() const
{
	return 0.0f;
}


bool GUI::captureWASDUPDOWN(int key, int action)
{
	if (key == GLFW_KEY_W) {
		if (fps_mode_)
			eye_ += zoom_speed_ * look_;
		else
			camera_distance_ -= zoom_speed_;
		return true;
	} else if (key == GLFW_KEY_S) {
		if (fps_mode_)
			eye_ -= zoom_speed_ * look_;
		else
			camera_distance_ += zoom_speed_;
		return true;
	} else if (key == GLFW_KEY_A) {
		if (fps_mode_)
			eye_ -= pan_speed_ * tangent_;
		else
			center_ -= pan_speed_ * tangent_;
		return true;
	} else if (key == GLFW_KEY_D) {
		if (fps_mode_)
			eye_ += pan_speed_ * tangent_;
		else
			center_ += pan_speed_ * tangent_;
		return true;
	} else if (key == GLFW_KEY_DOWN) {
		if (fps_mode_)
			eye_ -= pan_speed_ * up_;
		else
			center_ -= pan_speed_ * up_;
		return true;
	} else if (key == GLFW_KEY_UP) {
		if (fps_mode_)
			eye_ += pan_speed_ * up_;
		else
			center_ += pan_speed_ * up_;
		return true;
	}
	return false;
}


// Delegrate to the actual GUI object.
void GUI::KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
	GUI* gui = (GUI*)glfwGetWindowUserPointer(window);
	gui->keyCallback(key, scancode, action, mods);
}

void GUI::MousePosCallback(GLFWwindow* window, double mouse_x, double mouse_y)
{
	GUI* gui = (GUI*)glfwGetWindowUserPointer(window);
	gui->mousePosCallback(mouse_x, mouse_y);
}

void GUI::MouseButtonCallback(GLFWwindow* window, int button, int action, int mods)
{
	GUI* gui = (GUI*)glfwGetWindowUserPointer(window);
	gui->mouseButtonCallback(button, action, mods);
}

void GUI::MouseScrollCallback(GLFWwindow* window, double dx, double dy)
{
	GUI* gui = (GUI*)glfwGetWindowUserPointer(window);
	gui->mouseScrollCallback(dx, dy);
}
