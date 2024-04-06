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
		currentTheta.resize(7);
		//currentTheta.setZero();
		currentTheta << PI / 6, PI / 6, PI / 6, PI / 6, PI / 6, PI / 6, PI / 6;
		return 0;
	};
	void reset(double time);
	int command(int argc, myCONST_SPEC char** argv);

protected:

	Eigen::VectorXf currentTheta;

	double prevSec = 0.0;
	double prevTime = 0.0;
	double velocity = 0.1; // m/s (initial)
	double distance = 0;

	Character* character;
	Hermite* m_spline;

};