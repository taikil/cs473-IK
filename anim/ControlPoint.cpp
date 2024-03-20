#include "ControlPoint.h"

ControlPoint::ControlPoint(const std::string& name) : BaseSystem(name)
{
	m_pos = glm::dvec3(0.0, 0.0, 0.0);
	tangent = glm::dvec3(0.0, 0.0, 0.0);
}

// Copy constructor
ControlPoint::ControlPoint(const ControlPoint& other) : BaseSystem(other.m_name)
{
	m_pos = other.m_pos;
	tangent = other.tangent;
	std::copy(other.samplePoints, other.samplePoints + numSamples, samplePoints);
	m_model = other.m_model; // Assuming GLMmodel supports copy semantics
}

// Copy assignment operator
ControlPoint& ControlPoint::operator=(const ControlPoint& other)
{
	if (this != &other) // self-assignment check
	{
		m_name = other.m_name;
		m_pos = other.m_pos;
		tangent = other.tangent;
		std::copy(other.samplePoints, other.samplePoints + numSamples, samplePoints);
		m_model = other.m_model; // Assuming GLMmodel supports copy semantics
	}
	return *this;
}

void ControlPoint::getPos(glm::dvec3& pos)
{
	pos = m_pos;
}

void ControlPoint::getTan(glm::dvec3& tan)
{
	tan = tangent;
}

void ControlPoint::setPos(const glm::dvec3& pos)
{
	m_pos = pos;
}

void ControlPoint::setTan(const glm::dvec3& tan)
{
	tangent = tan;
}

void ControlPoint::getPoints(glm::dvec3 points[20])
{
	for (size_t i = 0; i < numSamples; i++) {
		points[i] = samplePoints[i];
	}
}


void ControlPoint::setSamplePoints(const glm::dvec3 points[20]) {
	for (size_t i = 0; i < numSamples; i++) {
		samplePoints[i] = points[i];
	}
}



void ControlPoint::reset(double time)
{
	m_pos = glm::dvec3(0.0, 0.0, 0.0);
	tangent = glm::dvec3(0.0, 0.0, 0.0);
}

void ControlPoint::readModel(const char* fname)
{
	m_model.ReadOBJ(fname);
	glmFacetNormals(&m_model);
	glmVertexNormals(&m_model, 90);
}

int ControlPoint::command(int argc, myCONST_SPEC char** argv)
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
			readModel(argv[1]);
			return TCL_OK;
		}
		else
		{
			animTcl::OutputMessage("Usage: read <file_name>");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "pos") == 0)
	{
		if (argc == 4)
		{
			m_pos.x = atof(argv[1]);
			m_pos.y = atof(argv[2]);
			m_pos.z = atof(argv[3]);
		}
		else
		{
			animTcl::OutputMessage("Usage: pos <x> <y> <z> ");
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

void ControlPoint::display(GLenum mode)
{
	glEnable(GL_LIGHTING);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glTranslated(m_pos.x, m_pos.y, m_pos.z);

	if (m_model.numvertices > 0)
		glmDraw(&m_model, GLM_SMOOTH | GLM_MATERIAL);
	else
		glutSolidSphere(1.0, 20, 20);

	glPopMatrix();
	glPopAttrib();
}
