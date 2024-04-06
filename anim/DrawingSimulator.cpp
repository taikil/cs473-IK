#include "DrawingSimulator.h"

DrawingSimulator::DrawingSimulator(const std::string& name, Character* target, Hermite* spline) :
	BaseSimulator(name),
	character(target),
	m_spline(spline)  // Add a member variable to store the spline
{
}

DrawingSimulator::~DrawingSimulator()
{
}

int DrawingSimulator::step(double time) // 0.01s
{
	if (time == 0.1) {
		velocity = 0.1;
		distance = 0.0;
		prevTime = 0.0;
		prevSec = 0.0;
	}
	double timeStep = time - prevTime;
	prevTime = time;

	if (time - prevSec >= 1.0) {
		animTcl::OutputMessage("The simulation time is %.3f, velocity is: %.3f m/s", time, velocity);
		prevSec = time;
	}

	distance += timeStep;
	distance = std::min(distance, 1.0);
	if (distance == 1.0) distance = 0;

	currentEndEffectorPos = character->computeHandPosition(currentTheta);
	if (splineLoaded) {
		VectorObj curTarget;
		m_spline->getPoint(curTarget, distance);
		targetPoint = Eigen::Vector3f(curTarget[0], curTarget[1], curTarget[2]); // Example goal point coordinates
	}
	else {
		targetPoint = Eigen::Vector3f(0, 0, 0);
	}

	Eigen::VectorXf newTheta;
	Eigen::MatrixXf curJacobian = character->computeJacobian(currentTheta);
	jacobian = curJacobian;

	character->IKSolver(jacobian, currentTheta, currentEndEffectorPos, targetPoint, newTheta);

	currentTheta = newTheta;

	character->setThetas(currentTheta);

	return 0;

}

void DrawingSimulator::reset(double time)
{

	// TODO: overload this function to specify how an object should be reset

	character->reset(0);

}

int DrawingSimulator::command(int argc, myCONST_SPEC char** argv)
{
	if (argc < 1)
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str());
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "read") == 0)
	{
		if (argc == 2)
		{
			glScalef(0.5, 0.5, 0.5);
			m_spline->loadFromFile2D(argv[1]);
			splineLoaded = true;
			currentTheta = Eigen::VectorXf(7);
			currentTheta << 0, 0, -PI / 4, 0, PI / 24, PI / 24, PI / 24;
			character->setThetas(currentTheta);
			Eigen::MatrixXf curJacobian = character->computeJacobian(currentTheta);
			jacobian = curJacobian;
			currentEndEffectorPos = character->computeHandPosition(currentTheta);
		}
		else
		{
			animTcl::OutputMessage("Usage: Unable to read text file");
			return TCL_ERROR;

		}
	}
	else if (strcmp(argv[0], "reset") == 0)
	{
		reset(0);
	}

	glutPostRedisplay();
	return TCL_OK;

}