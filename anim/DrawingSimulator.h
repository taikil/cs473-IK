#include <GLModel/GLModel.h>
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include "BaseSimulator.h"
#include "BaseSystem.h"
#include "Character.h"
#include "Hermite.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>



#include <string>

// a sample simulator

class DrawingSimulator : public BaseSimulator
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	DrawingSimulator(const std::string& name, Character* target, Hermite* spline);
	~DrawingSimulator();

	int step(double time);
	int init(double time)
	{
		Vector pos;
		character->getState(pos);
		m_pos0 = glm::dvec3(pos[0], pos[1], pos[2]);

		return 0;
	};

	int command(int argc, myCONST_SPEC char** argv);

protected:

	glm::dvec3 m_pos0; // initial position
	glm::dvec3 m_vel0; // initial velocity
	glm::dvec3 m_pos;
	glm::dvec3 m_vel;
	Eigen::VectorXf currentTheta;

    double prevSec = 0.0;
	double prevTime = 0.0;
	double velocity = 0.1; // m/s (initial)
	double distance = 0;

	Character* character;
	Hermite* m_spline;

};