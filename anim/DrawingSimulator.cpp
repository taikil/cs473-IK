#include "DrawingSimulator.h"

DrawingSimulator::DrawingSimulator(const std::string& name, BaseSystem* target, Hermite* spline) :
	BaseSimulator(name),
	m_object(target),
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
    // Use the Spline class to get the car's position along the spline


    // Update the car's position using the translate function

    // Assuming the car has a setState function to update its internal state
    //m_object->setState(pos);

    return 0;

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