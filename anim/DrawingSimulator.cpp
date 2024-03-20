#include "DrawingSimulator.h"

DrawingSimulator::DrawingSimulator(const std::string& name, BaseSystem* target, Spline* spline) :
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
    glm::dvec3 carPosition = m_spline->getCarPosition(velocity, timeStep, distance); // seperated them to compute velocity 
    glm::dvec3 carRotation = m_spline->getTangents(velocity, timeStep, distance);

    if (glm::length(carPosition) > 0.0) {
        // Calculate the rotation angle in degrees based on the arctangent of the tangent vector components
        double rotationAngleDegrees = glm::degrees(atan2(carRotation.x, carRotation.y));

        // Rotate the car around the vertical axis based on the calculated angle
        static_cast<Character*>(m_object)->rotate(glm::dvec3(0, 0, 1), rotationAngleDegrees);
    }

    // Update the car's position using the translate function
    static_cast<Character*>(m_object)->translate(carPosition);

    // Assuming the car has a setState function to update its internal state
    //m_object->setState(pos);

    return 0;

}