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

	Eigen::Vector3f targetPoint(0.0, 0.0, 0.0); // Example goal point coordinates

	Eigen::Vector3f currentEndEffectorPos = character->computeHandPosition(currentTheta);

	// Compute the IK solution to minimize the error
	Eigen::VectorXf newTheta;
	Eigen::MatrixXf jacobian = character->computeJacobian(currentTheta);

	character->IKSolver(jacobian, currentTheta, currentEndEffectorPos, targetPoint, newTheta);

	// Update the character's joint angles with the new solution
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