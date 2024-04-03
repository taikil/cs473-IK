#include "Character.h"

Character::Character(const std::string& name) :
	BaseSystem(name),
	Troot(0.0, 0.0, 2.0, 1.0),
	Tshoulder(0.666, 1.5, 0.0, 1.0),
	Telbow(2.0, 0.0, 0.0, 1.0),
	Twrist(2.0, 0.0, 0.0, 1.0),
	Phand(0.8, 0.0, 0.0, 1.0)

{
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
	double p[3] = { 0,0,0 };
	setState(p);

}	// Character::Reset


void Character::bob(Eigen::Vector4<float> translation) {
	Troot = translation;
}

void Character::rotate(Eigen::Vector3<float>, double angleDegrees)
{

}


int Character::command(int argc, myCONST_SPEC char** argv)
{
	if (argc < 1)
	{
		animTcl::OutputMessage("system %s: wrong number of params.", m_name.c_str());
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "translate") == 0)
	{
		if (argc == 4)
		{
			bob(Eigen::Vector4<float>(atof(argv[1]), atof(argv[2]), atof(argv[3]), 1));
		}
		else
		{
			animTcl::OutputMessage("Usage: translate <x> <y> <z> ");
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

	glPushMatrix();

	glPushMatrix();
	glScaled(0.1, 0.1, 0.1);
	glutSolidSphere(1, 20, 20);
	glPopMatrix();

	glTranslated(Tshoulder.x(), Tshoulder.y(), Tshoulder.z());
	glPushMatrix();
	glScaled(0.1, 0.1, 0.1);
	glutSolidSphere(1, 20, 20);
	glPopMatrix();

	glTranslated(Telbow.x(), Telbow.y(), Telbow.z());
	glPushMatrix();
	glScaled(0.1, 0.1, 0.1);
	glutSolidSphere(1, 20, 20);
	glPopMatrix();


	glTranslated(Twrist.x(), Twrist.y(), Twrist.z());
	glPushMatrix();
	glScaled(0.1, 0.1, 0.1);
	glutSolidSphere(1, 20, 20);
	glPopMatrix();

	glTranslated(Phand.x(), Phand.y(), Phand.z());
	glPushMatrix();
	glScaled(0.1, 0.1, 0.1);
	glutSolidSphere(1, 20, 20);
	glPopMatrix();

	glPopMatrix();

	for (int i = 0; i < 2; i++) {
		glPushMatrix();
		for (int j = 0; j < 3; j++) {
			glPushMatrix();
			{
				glTranslated(armPos(i, j), j == 0 ? 1.5 : 0, 0);
				Eigen::Vector3f distance;
				distance = Eigen::Vector3f(i == 0 ? -armLen[j] : armLen[j], 0, 0);
				float rotation = i == 0 ? 0 : 0; // Right hand positive rotations
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
		else if (i  == 1 || i == 4 || i == 5) // y-axis rotation
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

		// Compute the derivative of end effector position w.r.t. the current joint angle
		Eigen::Vector4f dEndEffectorPos_dTheta = transformation * rotationXDerivative(theta(i)) * Eigen::Vector4f::Zero();

		if (i == 0) // Troot
			dEndEffectorPos_dTheta.head<3>() += Troot.head<3>();
		else if (i == 3) // Telbow
			dEndEffectorPos_dTheta.head<3>() += Troot.head<3>() + Tshoulder.head<3>();
		else if (i == 6) // Twrist
			dEndEffectorPos_dTheta.head<3>() += Troot.head<3>() + Tshoulder.head<3>() + Telbow.head<3>();

		// Fill in the corresponding column of the Jacobian matrix **
		jacobian.col(i) = dEndEffectorPos_dTheta.head<3>();
		animTcl::OutputMessage("Jacobian[%d]: (%f, %f, %f)", i, jacobian(0, i), jacobian(1, i), jacobian(2, i));
	}
	return jacobian;
}

Eigen::MatrixXf Character::pseudoinverse(Eigen::MatrixXf jacobian) {
	Eigen::PartialPivLU<Eigen::MatrixXf> lu(jacobian);
	Eigen::VectorXf x = lu.solve(jacobian);
	return x;
}

void Character::IKSolver(const Eigen::MatrixXf& J, const Eigen::VectorXf& currentTheta, const Eigen::Vector3f& currentP, const Eigen::Vector3f& targetP, Eigen::VectorXf& newTheta) {
	Eigen::Vector3f err = targetP - currentP; // Compute the error
	Eigen::Vector3f pTargetP = 0.1 * err + currentP; // Compute the target position for the end effector

	IKSolve(J, currentTheta, currentP, pTargetP, newTheta); // Call the iterative IK solver
}

void Character::IKSolve(const Eigen::MatrixXf& J, const Eigen::VectorXf& currentTheta, const Eigen::Vector3f& currentP, const Eigen::Vector3f& targetP, Eigen::VectorXf& newTheta) {
	const float epsilon = 0.01; // Convergence threshold
	const float k = 0.1; // Step size factor

	Eigen::Vector3f err = targetP - currentP; // Compute the error

	do {
		Eigen::VectorXf dX = k * err; // Take a step to reduce the error
		Eigen::MatrixXf J_pseudo = pseudoinverse(J); // Compute the pseudoinverse of J
		Eigen::VectorXf dQ = J_pseudo * dX; // Compute change in joint angles

		newTheta = currentTheta + dQ; // Update joint angles
		Eigen::Vector3f P = computeHandPosition(newTheta); // Compute new end effector position

		err = targetP - P; // Compute new error
	} while (err.norm() > epsilon); // Repeat until error is below threshold
}

Eigen::Vector3f Character::computeHandPosition(const Eigen::VectorXf& theta)
{
	Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

	// Apply rotations and translations to compute the final transformation matrix
	for (int i = 0; i < theta.size(); ++i) {
		if (i == 0 || i == 3) // x-axis rotation
			transformation *= rotationX(theta(i));
		else if (i == 1 || i == 4 || i == 5) // y-axis rotation
			transformation *= rotationY(theta(i));
		else if (i == 2 || i == 6) // z-axis rotation
			transformation *= rotationZ(theta(i));

		// Apply translations
		if (i == 0) // Troot
			transformation *= translationMatrix(Troot.head<3>()); // Use only the translation part of Troot
		else if (i == 3) // Telbow
			transformation *= translationMatrix(Tshoulder.head<3>()); // Use only the translation part of Tshoulder
		else if (i == 6) // Twrist
			transformation *= translationMatrix(Telbow.head<3>()); // Use only the translation part of Telbow
	}

	// Compute the position of Phand by applying the final transformation to the origin
	Eigen::Vector4f PhandPosition = transformation * Eigen::Vector4f::Zero();

	// Return the position of Phand as a 3D vector
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

