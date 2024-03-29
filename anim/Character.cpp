#include "Character.h"

Character::Character(const std::string& name, Spline* spline) :
	BaseSystem(name),
	m_sx(1.0f),
	m_sy(1.0f),
	m_sz(1.0f),
	m_pos(0, 0, 0),
	m_spline(spline)  // Add a member variable to store the spline
{
	m_model.ReadOBJ("../Build/data/f-16.obj");
	glmUnitize(&m_model);
	glmFacetNormals(&m_model);
	glmVertexNormals(&m_model, 90);
	armPos << -1.666, -2.0, -1.4,
		1.666, 2.0, 1.4;
	armLen << 1.0, 1.0, 0.4;
	legPos << -0.5, 0.0, 0.0,
		0.5, 0.0, 0.0,
		-2.666, -2.0, -1.2;
}	// Character

void Character::getState(double* p)
{

	// Convert glm::dvec3 to double array
	p[0] = m_pos.x;
	p[1] = m_pos.y;
	p[2] = m_pos.z;
}	 //Character::getState

void Character::setState(double* p)
{
	glm::dvec3 position(p[0], p[1], p[2]);
	translate(position);


}	// Character::setState

void Character::reset(double time)
{
	double p[3] = { 0,0,0 };
	setState(p);

}	// Character::Reset


void Character::translate(glm::dvec3 translation) {
	m_pos = translation;
}

void Character::rotate(glm::dvec3 axis, double angleDegrees)
{
	// Reset to base orientation



}


void Character::readModel(const char* fname)
{
	m_model.ReadOBJ(fname);
	glmFacetNormals(&m_model);
	glmVertexNormals(&m_model, 90);
}

int Character::command(int argc, myCONST_SPEC char** argv)
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
	else if (strcmp(argv[0], "scale") == 0)
	{
		if (argc == 4)
		{
			m_sx = (float)atof(argv[1]);
			m_sy = (float)atof(argv[2]);
			m_sz = (float)atof(argv[3]);
		}
		else
		{
			animTcl::OutputMessage("Usage: scale <sx> <sy> <sz> ");
			return TCL_ERROR;

		}
	}
	else if (strcmp(argv[0], "translate") == 0)
	{
		if (argc == 4)
		{
			translate(glm::dvec3(atof(argv[1]), atof(argv[2]), atof(argv[3])));
		}
		else
		{
			animTcl::OutputMessage("Usage: translate <x> <y> <z> ");
			return TCL_ERROR;

		}
	}
	else if (strcmp(argv[0], "rotate") == 0)
	{
		if (argc == 5) // Need to pass axis and three rotation parameters
		{
			rotate(glm::dvec3(atof(argv[1]), atof(argv[2]), atof(argv[3])), atof(argv[4]));
		}
		else
		{
			animTcl::OutputMessage("Usage: rotate <axis_i> <axis_j> <axis_k> <angle_degrees>");
			return TCL_ERROR;

		}
	}
	else if (strcmp(argv[0], "flipNormals") == 0)
	{
		flipNormals();
		return TCL_OK;

	}
	else if (strcmp(argv[0], "load") == 0)
	{
		if (argc == 2)
		{
			return m_spline->load(argv[1]);
		}
		else
		{
			animTcl::OutputMessage("Usage: system <name> load \"<file name>\"");
			return TCL_ERROR;
		}
	}

	else if (strcmp(argv[0], "reset") == 0)
	{
		reset(0);
	}

	glutPostRedisplay();
	return TCL_OK;

}	// Character::command

void Character::drawCircleOutline(float r, int num_segments) {
	glBegin(GL_LINE_LOOP);
	glLineWidth(3.0f);
	for (int i = 0; i < num_segments; i++) {
		float theta = 2.0f * 3.1415926f * float(i) / float(num_segments);
		float x = r * cosf(theta);
		float y = r * sinf(theta);
		glVertex3f(x, y, 0);
	}
	glEnd();
}

void Character::drawBody() {
	glPushMatrix();
	{
		glScaled(1, 2, 1.0);
		drawCircleOutline(1.0, 20);
	}
	glPopMatrix();

	// Head
	glPushMatrix();
	{
		glTranslated(0, 2.5, 0);
		drawCircleOutline(0.5, 20);
	}
	glPopMatrix();
}

void Character::drawLegs() {
	for (int i = 0; i < 2; i++) {
		glPushMatrix();
		for (int j = 0; j < 3; j++) {
			glPushMatrix();
			{
				glTranslated(legPos(i, j), legPos(2, j), 0);
				glPushMatrix();
				{
					j == 2 ? glScaled(0.4, 0.2, 0) : glScaled(0.2, 1.0, 0);
					drawCircleOutline(1.0, 20);
				}
				glPopMatrix();
			}
		}
		glPopMatrix();
		glPopMatrix();
		glPopMatrix();
	}
	glPopMatrix();
	glPopMatrix();
}

void Character::drawArms() {
	glPushMatrix();
	glTranslated(5.46, 1.5, 0);
	glScaled(0.1, 0.1, 0.1);
	glutSolidSphere(1, 20, 20);
	glPopMatrix();

	for (int i = 0; i < 2; i++) {
		glPushMatrix();
		for (int j = 0; j < 3; j++) {
			glPushMatrix();
			{
				glTranslated(armPos(i, j), j == 0 ? 1.5 : 0, 0);
				Eigen::Vector3f distance;
				distance = Eigen::Vector3f(i == 0 ? -armLen[j] : armLen[j], 0, 0);
				float rotation = i == 0 ? -0 : 0;
				rotateFromBase(rotation, 0, 1, 0, distance);
				glPushMatrix();
				{
					j == 2 ? glScaled(0.4, 0.25, 0) : glScaled(1.0, 0.2, 0);
					drawCircleOutline(1.0, 20);
				}
				glPopMatrix();
			}
		}
		glPopMatrix();
		glPopMatrix();
		glPopMatrix();
	}
	glPopMatrix();
	glPopMatrix();
}

void Character::rotateFromBase(float angle, int x, int y, int z, Eigen::Vector3f distance) {
	glTranslatef(-distance.x(), -distance.y(), -distance.z());
	glRotatef(angle, x, y, z);
	//x == 1 ? rotationX(angle) : void(0);
	//y == 1 ? rotationY(angle) : void(0);
	//z == 1 ? rotationZ(angle) : void(0);
	glTranslatef(distance.x(), distance.y(), distance.z());
}

Eigen::Matrix4f Character::rotationX(float angle) {
	Eigen::Matrix4f rotX = Eigen::Matrix4f::Identity();
	rotX.block<3, 3>(0, 0) << 1, 0, 0,
		0, cos(angle), -sin(angle),
		0, sin(angle), cos(angle);
	//glMultMatrixd(rotX.data());
	return rotX;
}

Eigen::Matrix4f Character::rotationY(float angle) {
	Eigen::Matrix4f rotY = Eigen::Matrix4f::Identity();
	rotY.block<3, 3>(0, 0) << cos(angle), 0, sin(angle),
		0, 1, 0,
		-sin(angle), 0, cos(angle);
	return rotY;
}

Eigen::Matrix4f Character::rotationZ(float angle) {
	Eigen::Matrix4f rotZ = Eigen::Matrix4f::Identity();
	rotZ.block<3, 3>(0, 0) << cos(angle), -sin(angle), 0,
		sin(angle), cos(angle), 0,
		0, 0, 1;
	return rotZ;
}

Eigen::Matrix4f Character::rotationXDerivative(float angle) {
	Eigen::Matrix4f rotX = Eigen::Matrix4f::Identity();
	rotX.block<3, 3>(0, 0) << 1, 0, 0,
		0, -sin(angle), -cos(angle),
		0, cos(angle), -sin(angle);
	//glMultMatrixd(rotX.data());
	return rotX;
}

Eigen::Matrix4f Character::rotationYDerivative(float angle) {
	Eigen::Matrix4f rotY = Eigen::Matrix4f::Identity();
	rotY.block<3, 3>(0, 0) << -sin(angle), 0, cos(angle),
		0, 1, 0,
		-cos(angle), 0, -sin(angle);
	return rotY;
}

Eigen::Matrix4f Character::rotationZDerivative(float angle) {
	Eigen::Matrix4f rotZ = Eigen::Matrix4f::Identity();
	rotZ.block<3, 3>(0, 0) << -sin(angle), -cos(angle), 0,
		cos(angle), -sin(angle), 0,
		0, 0, 1;
	return rotZ;
}

Eigen::MatrixXf Character::computeJacobian(const std::vector<float>& theta) {
	int numThetas = theta.size();
	Eigen::MatrixXf jacobian(3, numThetas);

	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

	// Compute the Jacobian matrix
	for (int i = 0; i < numThetas; ++i) {
		if (i == 0 || i == 3) // x-axis rotation
			transformation *= rotationX(theta[i]);
		else if (i % 7 == 1 || i == 4 || i == 5) // y-axis rotation
			transformation *= rotationY(theta[i]);
		else if (i % 7 == 2) // z-axis rotation
			transformation *= rotationZ(theta[i]);
		Eigen::Vector4f endEffectorPos = transformation * Eigen::Vector4f::Zero();

		// Compute the derivative of end effector position w.r.t. the current joint angle
		Eigen::Vector4f dEndEffectorPos_dTheta = transformation * rotationXDerivative(theta[i]) * Eigen::Vector4f::Zero();

		// Fill in the corresponding column of the Jacobian matrix **
		jacobian.col(i) = dEndEffectorPos_dTheta.head<3>();
	}

	return jacobian;
}

Eigen::MatrixXf Character::pseudoinverse(Eigen::MatrixXf jacobian, Eigen::VectorXf p) {
	Eigen::PartialPivLU<Eigen::MatrixXf> lu(jacobian);
	//Eigen::VectorXf x = lu.solve(b);
}


void Character::display(GLenum mode)
{
	glEnable(GL_LIGHTING);
	glMatrixMode(GL_MODELVIEW);
	glEnable(GL_COLOR_MATERIAL);
	glColor3f(0.5, 0.1, 0.7);
	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glTranslated(m_pos[0], m_pos[1], m_pos[2]);
	glScalef(m_sx, m_sy, m_sz);
	//glm::mat4 rotationMatrix = glm::mat4_cast(m_rot);
	//glMultMatrixf(glm::value_ptr(rotationMatrix));

	glPushMatrix();
	{
		drawBody();

		drawArms();

		drawLegs();

	}
	glPopMatrix();

	glPopMatrix();
	glPopAttrib();

}	// Character::display

