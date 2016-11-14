#include "gui.h"
#include "config.h"
#include <jpegio.h>
#include "bone_geometry.h"
#include <iostream>
#include <debuggl.h>
#include <glm/gtc/matrix_access.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <cmath>


namespace {
	// Intersect a cylinder with radius 1/2, height 1, with base centered at
	// (0, 0, 0) and up direction (0, 1, 0).
	bool IntersectCylinder(const glm::vec3& origin, const glm::vec3& direction,
			float radius, float height, std::vector<float> t, glm::vec3 center)
	{
		//FIXME perform proper ray-cylinder collision detection

		// std::cout<<"\nCenter: "<<center.x<<" "<<center.y<<" "<<center.z;
		double a = pow(direction.x - origin.x,2) + pow(direction.z - origin.z,2);
		double b = 2*(direction.x - origin.x)*(origin.x - center.x) + 2*(direction.z - origin.z)*(origin.z - center.z);
		double c = pow(origin.x - center.x,2) + pow(origin.z - center.z,2) - pow(radius,2);
		double discrim = pow(b,2) - 4*a*c;
		if(discrim < 0)
		{
			// std::cout<<"\nDiscrim is nan!!";
			return false;
		}
		// std::cout<<"\na, b, c: "<<a<<" "<<b<<" "<<c;
		// std::cout<<"\ndiscrim: "<<discrim;
		discrim = sqrt(discrim);
		float t1 = (-1*b + discrim)/(2*a);
		float t2 = (-1*b - discrim)/(2*a);
		t.push_back(t1);
		t.push_back(t2);
		// std::cout<<"\nt-values: "<<t[0]<<" "<<t[1];
		// if(t[0] > 0 && t[0] < 1)
		// {
		// 	// std::cout<<"\nt-values: "<<t[0];
		// 	return true;
		// }
		// if(t[1] > 0 && t[1] < 1)
		// {
		// 	// std::cout<<"\nt-values: "<<t[1];
		// 	return true;
		// }
		glm::vec3 check1 = glm::vec3(origin + (t1*direction));
		std::cout<<"\ncheck1: "<<check1.x<<" "<<check1.y<<" "<<check1.z;
		glm::vec3 check2 = glm::vec3(origin + (t2*direction));
		std::cout<<"\ncheck2: "<<check2.x<<" "<<check2.y<<" "<<check2.z;
		std::cout<<"\n\n";

		if(check1.y > 0 && check1.y < height)
		{
			return true;
		}
		if(check2.y > 0 && check2.y < height)
		{
			return true;
		}
		// std::cout<<"\nt0 and t1: "<<t[0]<<" "<<t[1];

		return false;
	}
}

GUI::GUI(GLFWwindow* window)
	:window_(window)
{
	glfwSetWindowUserPointer(window_, this);
	glfwSetKeyCallback(window_, KeyCallback);
	glfwSetCursorPosCallback(window_, MousePosCallback);
	glfwSetMouseButtonCallback(window_, MouseButtonCallback);

	glfwGetWindowSize(window_, &window_width_, &window_height_);
	float aspect_ = static_cast<float>(window_width_) / window_height_;
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
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
		glfwSetWindowShouldClose(window_, GL_TRUE);
		return ;
	}
	if (key == GLFW_KEY_J && action == GLFW_RELEASE) {
		//DONE
		GLubyte* pixels = new GLubyte[3 * window_width_* window_height_];
		glReadPixels(0, 0, window_width_, window_height_, GL_RGB, GL_UNSIGNED_BYTE, pixels);
		SaveJPEG("jpg1",window_width_, window_height_, pixels);
	}

	if (captureWASDUPDOWN(key, action))
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
	glm::vec3 mouse_direction = glm::normalize(glm::vec3(delta_x, delta_y, 0.0f));
	glm::vec2 mouse_start = glm::vec2(last_x_, last_y_);
	glm::vec2 mouse_end = glm::vec2(current_x_, current_y_);
	glm::uvec4 viewport = glm::uvec4(0, 0, window_width_, window_height_);

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
		return ;
	}

	// FIXME: highlight bones that have been moused over
	glm::vec3 p = glm::vec3(mouse_x, mouse_y, 0);
	p = glm::unProject(p, model_matrix_, projection_matrix_, viewport);
	glm::vec3 q = glm::vec3(mouse_x, mouse_y, 1);
	q = glm::unProject(q, model_matrix_, projection_matrix_, viewport);
	glm::vec3 ray = p - q;
	glm::vec4 t_origin;
	glm::vec4 t_ray;

for(int i = 0; i < mesh_->skeleton.numJoints; i++)
	{
		Joint j = mesh_->skeleton.joints[i];
		t_origin = j.rot * glm::vec4(p,1);
		t_ray = j.rot * glm::vec4(ray,0);
		glm::vec3 origin = glm::vec3(t_origin.x, t_origin.y, t_origin.z);
		glm::vec3 direction = glm::vec3(t_ray.x, t_ray.y, t_ray.z);

		// std::cout<<"\norigin   : "<<origin.x<<" "<<origin.y<<" "<<origin.z;
		// std::cout<<"\ndirection: "<<direction.x<<" "<<direction.y<<" "<<direction.z;
		// std::cout<<"\n\n";
		std::vector<float> t;
		float len = (float) j.length;
		int collision = IntersectCylinder(origin, direction, kCylinderRadius, len, t, j.check);
		if(collision == 1)
		{
			std::cout<<"\nHIT A BONE!!!: "<<i;
			break;
		}
	}


	current_bone_ = -1;
}


void GUI::mouseButtonCallback(int button, int action, int mods)
{
	drag_state_ = (action == GLFW_PRESS);
	current_button_ = button;
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

	aspect_ = static_cast<float>(window_width_) / window_height_;
	projection_matrix_ =
		glm::perspective((float)(kFov * (M_PI / 180.0f)), aspect_, kNear, kFar);
	model_matrix_ = glm::mat4(1.0f);
}

MatrixPointers GUI::getMatrixPointers() const
{
	MatrixPointers ret;
	ret.projection = &projection_matrix_[0][0];
	ret.model= &model_matrix_[0][0];
	ret.view = &view_matrix_[0][0];
	return ret;
}

bool GUI::setCurrentBone(int i)
{
	if (i < 0 || i >= mesh_->getNumberOfBones())
		return false;
	current_bone_ = i;
	return true;
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
