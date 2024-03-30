#pragma once
#ifndef MY_CHARACTER_H
#define MY_CHARACTER_H


#include "BaseSystem.h"
#include <shared/defs.h>
#include <windows.h>
#include <util/util.h>
#include "animTcl.h"
#include "Hermite.h"

#ifdef Success
#undef Success
#endif
#include <Eigen/Dense>

#include <GLmodel/GLmodel.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/quaternion.hpp>
#include "shared/opengl.h"

// a sample system
class Character : public BaseSystem
{

public:
	Character(const std::string& name);
	virtual void getState(double* p);
	virtual void setState(double* p);
	void reset(double time);

	void translate(glm::dvec3 translation);
	void rotate(glm::dvec3 axis, double angleDegrees);

	void display(GLenum mode = GL_RENDER);

	void readModel(const char* fname);
	void Character::drawBoard();
	int command(int argc, myCONST_SPEC char** argv);
	void rotateFromBase(float angle, int x, int y, int z, Eigen::Vector3f distance);
	void drawCircleOutline(float r, int num_segments);
	void drawBody();
	void drawArms();
	void drawLegs();


	Eigen::Matrix4f rotationX(float angle);
	Eigen::Matrix4f rotationY(float angle);
	Eigen::Matrix4f rotationZ(float angle);
	Eigen::Matrix4f rotationXDerivative(float angle);
	Eigen::Matrix4f rotationYDerivative(float angle);
	Eigen::Matrix4f rotationZDerivative(float angle);
	Eigen::MatrixXf computeJacobian(const std::vector<float>& theta);
	Eigen::MatrixXf pseudoinverse(Eigen::MatrixXf jacobian, Eigen::VectorXf p);

protected:

	float m_sx;
	float m_sy;
	float m_sz;

	glm::dvec3 m_pos;

	Eigen::Matrix<float, 2, 3> armPos;
	Eigen::Vector3<float> armLen;
	Eigen::Matrix<float, 3, 3> legPos;

	//Arm Length: 1.0, 1.0, 0.4
	//Start Position = 
	Eigen::Matrix<float, 3, 7> jacobian;

	GLMmodel m_model;

};
#endif


