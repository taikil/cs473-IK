#pragma once
#ifndef MY_CHARACTER_H
#define MY_CHARACTER_H


#include "BaseSystem.h"
#include <shared/defs.h>
#include <windows.h>
#include <util/util.h>
#include "animTcl.h"
#include "Spline.h"

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
	Character(const std::string& name, Spline* spline);
	virtual void getState(double* p);
	virtual void setState(double* p);
	void reset(double time);

	void translate(glm::dvec3 translation);
	void rotate(glm::dvec3 axis, double angleDegrees);

	void display(GLenum mode = GL_RENDER);

	void readModel(const char* fname);
	void flipNormals(void) { glmReverseWinding(&m_model); }
	int command(int argc, myCONST_SPEC char** argv);
	void drawCircleOutline(float r, int num_segments);
	void drawBody();
	void drawArms();
	void drawLegs();

protected:

	float m_sx;
	float m_sy;
	float m_sz;

	glm::dvec3 m_pos;

	Eigen::Matrix<float, 2, 3> armPos;
	Eigen::Matrix<float, 2, 3> legPos;

	GLMmodel m_model;
	Spline* m_spline;

};
#endif


