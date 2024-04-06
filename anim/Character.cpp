#include "Character.h"

Character::Character(const std::string& name) :
	BaseSystem(name),
	Troot(0.0, 0.0, 2.0, 1.0),
	Tshoulder(0.666, 1.5, 0.0, 1.0),
	Telbow(2.0, 0.0, 0.0, 1.0),
	Twrist(2.0, 0.0, 0.0, 1.0),
	Phand(0.8, 0.0, 0.0, 1.0)

{
	thetas = Eigen::VectorXf(7);
	thetas << 0, 0, 0, 0, 0, 0, 0;
	//thetas << PI / 12, PI / 12, PI / 12, PI / 12, PI / 12, PI / 12, PI / 12;
	armPos << -1.666, -2.0, -1.4,
		1.666, 2.0, 1.4;

	armLen << 1.0, 1.0, 0.4;

	legPos << -0.5, 0.0, 0.0,
		0.5, 0.0, 0.0,
		-2.666, -2.0, -1.0;
}	// Character

void Character::getState(double* p)
{

	p[0] = Phand.x();
	p[1] = Phand.y();
	p[2] = Phand.z();
}	 //Character::getState

void Character::setState(double* p)
{
	Eigen::Vector4<float> position(p[0], p[1], p[2], 1);
	bob(position);


}	// Character::setState

void Character::reset(double time)
{
	bob(Eigen::Vector4f(0, 0, 2, 1));
}	// Character::Reset


void Character::setThetas(Eigen::MatrixXf newTheta) {
	thetas = newTheta;
	glutPostRedisplay();
}

void Character::bob(Eigen::Vector4f translation) {
	Troot = translation;
	glutPostRedisplay();
}


int Character::command(int argc, myCONST_SPEC char** argv)
{
	if (argc < 1)
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str());
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "position") == 0)
	{
		if (argc == 4)
		{
			bob(Eigen::Vector4<float>(atof(argv[1]), atof(argv[2]), atof(argv[3]), 1));
		}
		else
		{
			animTcl::OutputMessage("Usage: position <x> <y> <z> ");
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
	glLineWidth(8.0f);
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
					if (j == 2) rotateFromBase(-100, 1, 0, 0, Eigen::Vector3f(0, 0.1, 0));
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
	for (int i = 0; i < 2; i++) {
		glPushMatrix();
		for (int j = 0; j < 3; j++) {
			glPushMatrix();
			{
				glTranslated(armPos(i, j), j == 0 ? 1.5 : 0, 0);
				Eigen::Vector3f distance;
				distance = Eigen::Vector3f(i == 0 ? -armLen(j) : armLen(j), 0, 0);
				if (i == 1) {
					if (j == 0) {
						rotateFromBase(thetas(0), 1, 0, 0, distance); // x
						rotateFromBase(thetas(1), 0, 1, 0, distance); // y 
						rotateFromBase(thetas(2), 0, 0, 1, distance); // z
					}
					else if (j == 1) {
						rotateFromBase(thetas(3), 1, 0, 0, distance); // x
						rotateFromBase(thetas(4), 0, 1, 0, distance); // y
					}
					else {
						rotateFromBase(thetas(5), 0, 1, 0, distance); // y
						rotateFromBase(thetas(6), 0, 0, 1, distance); // z
					}
				}
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
	float degrees = angle * (180 / PI);

	glRotatef(degrees, x, y, z);
	glTranslatef(distance.x(), distance.y(), distance.z());
}

Eigen::Matrix4f Character::rotationX(float angle) {
	Eigen::Matrix4f rotX = Eigen::Matrix4f::Identity();
	rotX.block<3, 3>(0, 0) << 1, 0, 0,
		0, cos(angle), -sin(angle),
		0, sin(angle), cos(angle);
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
	rotX.block<3, 3>(0, 0) << 0, 0, 0,
		0, -sin(angle), -cos(angle),
		0, cos(angle), -sin(angle);
	//glMultMatrixd(rotX.data());
	return rotX;
}

Eigen::Matrix4f Character::rotationYDerivative(float angle) {
	Eigen::Matrix4f rotY = Eigen::Matrix4f::Identity();
	rotY.block<3, 3>(0, 0) << -sin(angle), 0, cos(angle),
		0, 0, 0,
		-cos(angle), 0, -sin(angle);
	return rotY;
}

Eigen::Matrix4f Character::rotationZDerivative(float angle) {
	Eigen::Matrix4f rotZ = Eigen::Matrix4f::Identity();
	rotZ.block<3, 3>(0, 0) << -sin(angle), -cos(angle), 0,
		cos(angle), -sin(angle), 0,
		0, 0, 0;
	return rotZ;
}

Eigen::MatrixXf Character::computeJacobian(const Eigen::MatrixXf theta) {
	int numThetas = theta.size();
	Eigen::MatrixXf jacobian(3, numThetas);

	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

	// Compute the Jacobian matrix
	for (int i = 0; i < numThetas; ++i) {
		if (i == 0 || i == 3) // x-axis rotation
			transformation *= rotationX(theta(i));
		else if (i == 1 || i == 4 || i == 5) // y-axis rotation
			transformation *= rotationY(theta(i));
		else if (i == 2 || i == 6) // z-axis rotation
			transformation *= rotationZ(theta(i));
		Eigen::Vector4f endEffectorPos = transformation * Eigen::Vector4f::Zero();

		// Apply translations
		if (i == 0) // Troot
			endEffectorPos += Troot;
		else if (i == 3) // Telbow
			endEffectorPos += Troot + Tshoulder;
		else if (i == 6) // Twrist
			endEffectorPos += Troot + Tshoulder + Telbow;

		Eigen::Vector4f dEndEffectorPos_dTheta = transformation * rotationXDerivative(theta(i)) * Eigen::Vector4f::Zero();

		if (i == 0) // Troot
			dEndEffectorPos_dTheta.head<3>() += Troot.head<3>();
		else if (i == 3) // Telbow
			dEndEffectorPos_dTheta.head<3>() += Troot.head<3>() + Tshoulder.head<3>();
		else if (i == 6) // Twrist
			dEndEffectorPos_dTheta.head<3>() += Troot.head<3>() + Tshoulder.head<3>() + Telbow.head<3>();

		jacobian.col(i) = dEndEffectorPos_dTheta.head<3>();
		//animTcl::OutputMessage("Jacobian[%d]: (%f, %f, %f)", i, jacobian(0, i), jacobian(1, i), jacobian(2, i));
	}

	//for (int i = 0; i < jacobian.rows(); ++i) {
	//	for (int j = 0; j < jacobian.cols(); ++j) {
	//		animTcl::OutputMessage("Jacobian(%d, %d): %f", i, j, jacobian(i, j));
	//	}
	//}

	return jacobian;
}

Eigen::MatrixXf Character::pseudoinverse(Eigen::MatrixXf jacobian) {
	Eigen::PartialPivLU<Eigen::MatrixXf> lu(jacobian);
	Eigen::VectorXf x = lu.solve(jacobian);
	return x;
}

void Character::IKSolver(Eigen::MatrixXf& J, Eigen::VectorXf& currentTheta, Eigen::Vector3f& currentP, Eigen::Vector3f& targetP, Eigen::VectorXf& newTheta) {
	Eigen::Vector3f err = targetP - currentP;
	Eigen::Vector3f pTargetP = (0.1 * err) + currentP;
	IKSolveTranspose(J, currentTheta, currentP, pTargetP, newTheta);
}

void Character::IKSolve(Eigen::MatrixXf& J, Eigen::VectorXf& currentTheta, Eigen::Vector3f& currentP, Eigen::Vector3f& targetP, Eigen::VectorXf& newTheta) {
	const float epsilon = 0.01;
	const float k = 0.1;

	Eigen::Vector3f err = targetP - currentP;

	do {
		Eigen::Vector3f dX = k * err;
		Eigen::MatrixXf J_pseudo = pseudoinverse(J);
		Eigen::VectorXf dQ = J_pseudo * dX;

		newTheta = currentTheta + dQ;
		Eigen::Vector3f P = computeHandPosition(newTheta);

		err = targetP - P;
	} while (err.norm() > epsilon);
}

void Character::IKSolveTranspose(Eigen::MatrixXf& J, Eigen::VectorXf& currentTheta, Eigen::Vector3f& currentP, Eigen::Vector3f& targetP, Eigen::VectorXf& newTheta) {
	const float epsilon = 0.01;
	const float k = 0.1;
	const int maxIterations = 100;

	Eigen::Vector3f err = targetP - currentP;

	int iter = 0;
	while (err.norm() > epsilon && iter < maxIterations) {
		Eigen::Vector3f dX = k * err;
		Eigen::MatrixXf J_transpose = J.transpose();

		Eigen::VectorXf dQ = J_transpose * dX;

		newTheta = currentTheta + dQ;

		Eigen::Vector3f newP = computeHandPosition(newTheta);


		err = targetP - newP;
		J = computeJacobian(newTheta);
		iter++;
	}
}

Eigen::Vector3f Character::computeHandPosition(const Eigen::VectorXf& theta)
{
	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();


	for (int i = 0; i < theta.rows(); ++i) {
		if (i == 0) // Troot
			transformation *= translationMatrix(Troot.head<3>());
		else if (i == 3) // Telbow
			transformation *= translationMatrix(Tshoulder.head<3>());
		else if (i == 6) // Twrist
			transformation *= translationMatrix(Telbow.head<3>());

		if (i == 0 || i == 3) // x-axis rotation
			transformation *= rotationX(theta(i));
		else if (i == 1 || i == 4 || i == 5) // y-axis rotation
			transformation *= rotationY(theta(i));
		else if (i == 2 || i == 6) // z-axis rotation
			transformation *= rotationZ(theta(i));
	}

	Eigen::Vector4f PhandPosition = transformation * Eigen::Vector4f(0, 0, 0, 1);

	return PhandPosition.head<3>();
}

Eigen::Matrix4f Character::translationMatrix(const Eigen::Vector3f& translationVector)
{
	Eigen::Matrix4f translationMat = Eigen::Matrix4f::Identity();
	translationMat.block<3, 1>(0, 3) = translationVector;
	return translationMat;
}

void drawSquare(float x, float y, float z, float length) {
	float halfLen = length / 2.0f;

	float x1 = x - halfLen;
	float x2 = x + halfLen;
	float y1 = y - halfLen;
	float y2 = y + halfLen;

	// Draw the square
	glBegin(GL_QUADS);
	glVertex3f(x1, y1, z);
	glVertex3f(x2, y1, z);
	glVertex3f(x2, y2, z);
	glVertex3f(x1, y2, z);
	glEnd();
}


void Character::drawBoard() {
	glColor3f(1.0, 1.0, 1.0);
	drawSquare(0, 0.0, -0.1, 12.0);
	glColor3f(0.0, 0.3, 0.15);
	glPushMatrix();
	{
		glScaled(1.5, 1, 0);
		drawSquare(0, 0.0, 0.0, 5);
	}
	glPopMatrix();
	glPushMatrix();
	{
		glColor3f(0.541, 0.314, 0.039);
		glTranslatef(0, -6, 5.99);
		glRotatef(90, 1, 0, 0);
		drawSquare(0, 0.0, 0.0, 12.0);
	}
	glPopMatrix();
}


void Character::display(GLenum mode)
{
	glEnable(GL_LIGHTING);
	glMatrixMode(GL_MODELVIEW);
	glEnable(GL_COLOR_MATERIAL);
	glColor3f(0.5, 0.1, 0.7);
	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);



	glPushMatrix();
	{
		drawBoard();
	}
	glPopMatrix();
	glPushMatrix();
	{
		glTranslated(Troot.x(), Troot.y(), Troot.z());
		drawBody();

		drawArms();

		drawLegs();

	}
	glPopMatrix();

	glPopMatrix();
	glPopAttrib();

}	// Character::display


