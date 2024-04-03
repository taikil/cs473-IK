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

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	Character(const std::string& name);
	virtual void getState(double* p);
	virtual void setState(double* p);
	void reset(double time);

	void bob(Eigen::Vector4<float> translation);
	void rotate(Eigen::Vector3<float> axis, double angleDegrees);

	void display(GLenum mode = GL_RENDER);

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
	Eigen::MatrixXf computeJacobian(const Eigen::MatrixXf theta);
	Eigen::MatrixXf pseudoinverse(Eigen::MatrixXf jacobian);
	void Character::IKSolver(const Eigen::MatrixXf& J, const Eigen::VectorXf& currentTheta, const Eigen::Vector3f& currentP, const Eigen::Vector3f& targetP, Eigen::VectorXf& newTheta);
	void Character::IKSolve(const Eigen::MatrixXf& J, const Eigen::VectorXf& currentTheta, const Eigen::Vector3f& currentP, const Eigen::Vector3f& targetP, Eigen::VectorXf& newTheta);
	Eigen::Vector3f computeHandPosition(const Eigen::VectorXf& theta);
	Eigen::Matrix4f translationMatrix(const Eigen::Vector3f& translationVector);


protected:

	Eigen::Matrix<float, 2, 3> armPos;
	//Arm Length: 1.0, 1.0, 0.4
	Eigen::Vector3<float> armLen;
	Eigen::Matrix<float, 3, 3> legPos;

	Eigen::Vector4<float> Troot;
	Eigen::Vector4<float> Tshoulder;
	Eigen::Vector4<float> Telbow;
	Eigen::Vector4<float> Twrist;
	Eigen::Vector4<float> Phand;


	//std::vector<float> theta;

	Eigen::Matrix<float, 3, 7> jacobian;


};
#endif


