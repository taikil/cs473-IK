#include "Spline.h"

Spline::Spline(const std::string& name) :
	BaseSystem(name),
	points(),
	arcLengths(0)
{
}

void Spline::getState(double* p) {

}


void Spline::setState(double* p) {

}


void Spline::initHermite()
{
	animTcl::OutputMessage("Initializing Hermite");
	// Ensure there are at least two control points
	if (points.size() < 2)
	{
		animTcl::OutputMessage("Error: Hermite spline requires at least two control points.");
		return;
	}

	// Iterate over each pair of consecutive control points
	for (size_t i = 0; i < points.size() - 1; ++i)
	{
		double stepSize = 1.0 / double(numSamples);
		ControlPoint p0 = points[i];
		ControlPoint p1 = points[i + 1];
		glm::dvec3 samplePoints[20];
		//for each point, run
		for (size_t j = 0; j < numSamples; j++) {
			double t = j * stepSize;
			samplePoints[j][0] = evaluateCurve(0, t, p0, p1);
			samplePoints[j][1] = evaluateCurve(1, t, p0, p1);
			samplePoints[j][2] = evaluateCurve(2, t, p0, p1);
			//animTcl::OutputMessage("Points %.3f / 20: x: %.3f, y: %.3f", static_cast<double>(j), samplePoints[j][0], samplePoints[j][1]);
		}
		points[i].setSamplePoints(samplePoints);

	}

	buildArcLengthLookupTable();

	// Redisplay if needed
	glutPostRedisplay();
}

double Spline::evaluateCurve(int d, double t, ControlPoint p0, ControlPoint p1) {
	glm::dvec3 pos0, pos1, tan0, tan1;
	p0.getPos(pos0);
	p0.getTan(tan0);
	p1.getPos(pos1);
	p1.getTan(tan1);
	double h1 = (2 * pow(t, 3) - 3 * pow(t, 2) + 1) * pos0[d]; // (2t^3 -3t^2 + 1)y0
	double h2 = (-2 * pow(t, 3) + 3 * pow(t, 2)) * pos1[d]; // (-2t^3 + 3t^2)y1
	double h3 = (pow(t, 3) - 2 * pow(t, 2) + t) * tan0[d]; // (t^3 - 2t^2+ t)s0
	double h4 = (pow(t, 3) - pow(t, 2)) * tan1[d]; // (t^3 - t^2)s1

	return (h1 + h2 + h3 + h4);

}

void Spline::addPoint(const glm::dvec3& pos, const glm::dvec3& tan) {
	if (points.size() < maxPoints) {
		numPoints++;
		ControlPoint new_point("Default");
		new_point.setPos(pos);
		new_point.setTan(tan);
		points.push_back(new_point);

		// Output characteristics of the added point
		animTcl::OutputMessage("Added Point:");
		animTcl::OutputMessage("  Position: %.3f %.3f %.3f", pos.x, pos.y, pos.z);
		animTcl::OutputMessage("  Tangent: %.3f %.3f %.3f", tan.x, tan.y, tan.z);
	}
	else {
		animTcl::OutputMessage("Error: Maximum number of control points reached (40).");
	}
}

void Spline::catMullRom() {
	//First Point
	if (points.size() > 3) {
		ControlPoint& p0 = points[0];
		ControlPoint& p1 = points[1];
		ControlPoint& p2 = points[2];
		setEndPointTangent(p0, p1, p2, false);
	}



	for (size_t i = 1; i < points.size() - 1; i++) {
		ControlPoint& p0 = points[i - 1];
		ControlPoint& p1 = points[i];
		ControlPoint& p2 = points[i + 1];
		setTangent(p0, p1, p2);
	}
	int size = points.size() - 1; // last index
	ControlPoint& p0 = points[size - 2];
	ControlPoint& p1 = points[size - 1];
	ControlPoint& p2 = points[size];
	setEndPointTangent(p0, p1, p2, true);

	initHermite();
}

int Spline::setTangent(ControlPoint& a, ControlPoint& b, ControlPoint& c)
{
	glm::dvec3 pos_A, pos_C;
	a.getPos(pos_A);
	c.getPos(pos_C);
	glm::dvec3 tangent = (pos_C - pos_A) / 2.0;
	b.setTan(tangent);
	//animTcl::OutputMessage("  New tangent: %.3f %.3f %.3f", tangent.x, tangent.y, tangent.z);

	return TCL_OK;
}

int Spline::setEndPointTangent(ControlPoint& a, ControlPoint& b, ControlPoint& c, bool end) {
	glm::dvec3 pos_A, pos_B, pos_C;
	a.getPos(pos_A);
	b.getPos(pos_B);
	c.getPos(pos_C);
	glm::dvec3 tangent;
	// 2(p1-p0) - (p2-p0)/2
	if (end) {
		tangent = (2.0 * (pos_C - pos_B)) - ((pos_C - pos_A) * 0.5);
		b.setTan(tangent);
		c.setTan(tangent);
	}
	else {
		tangent = (2.0 * (pos_B - pos_A)) - ((pos_C - pos_A) * 0.5);
		a.setTan(tangent);
	}
	return TCL_OK;
}

void Spline::reset(double time)
{
	initHermite();
	//points.clear();
	//numPoints = 0;
}

double Spline::getArcLength(const ControlPoint& p0, const ControlPoint& p1) {
	double stepSize = 1.0 / double(numSamples);
	double arcLength = 0.0;

	for (int i = 0; i < numSamples; ++i) {
		double t0 = i * stepSize; // range(0,1)
		double t1 = (i + 1) * stepSize;

		// u(t1) - u(t0)
		double deltaS = glm::length(glm::dvec3(
			evaluateCurve(0, t1, p0, p1) - evaluateCurve(0, t0, p0, p1),
			evaluateCurve(1, t1, p0, p1) - evaluateCurve(1, t0, p0, p1),
			evaluateCurve(2, t1, p0, p1) - evaluateCurve(2, t0, p0, p1)
		));
		double accumulatedLength = arcLengths.size() > 0 ? arcLengths.back().arcLength : 0.0;
		ArcLengthEntry entry;
		entry.arcLength = deltaS + accumulatedLength;
		entry.t = t0;
		entry.p0 = p0;
		entry.p1 = p1;
		arcLengths.push_back(entry);
		arcLength += deltaS;
		//animTcl::OutputMessage("  New len: %.3f, new t: %.3f", deltaS + accumulatedLength, t0);
	}

	return arcLength;
}

void Spline::buildArcLengthLookupTable() {
	arcLengths.clear();
	for (size_t i = 0; i < points.size() - 1; ++i) {
		double segmentLength = getArcLength(points[i], points[i + 1]); //Length between segments
		//double accumulatedLength = arcLengths.back();

		//arcLengths.push_back(segmentLength + accumulatedLength);
	}
	animTcl::OutputMessage("  Total entries in arcLengths: %zu", arcLengths.size()); // Output total entries
}

double Spline::getSplineLength(double t) {

	// Check if the arc lengths lookup table is empty
	if (arcLengths.empty()) {
		animTcl::OutputMessage("Error: Arc lengths lookup table is not initialized.");
		return 0.0;
	}

	// Ensure parameter is within the valid range [0, 1]
	t = std::max(0.0, std::min(1.0, t));
	//t = 1 / t;

	// Map parameter value to arc length using the lookup table
	double targetLength = t * arcLengths.back().arcLength;

	// Find the segment that contains the target arc length
	size_t segmentIndex = 0;
	while (segmentIndex < arcLengths.size() - 2 && arcLengths[segmentIndex].arcLength < targetLength) {
		++segmentIndex;
	}

	// Interpolate within the segment to find the corresponding arc length
	double val = (targetLength - arcLengths[segmentIndex].arcLength) / (arcLengths[segmentIndex + 1].arcLength - arcLengths[segmentIndex].arcLength);

	// Return the interpolated arc length
	return arcLengths[segmentIndex].arcLength + val * (arcLengths[segmentIndex + 1].arcLength - arcLengths[segmentIndex].arcLength);
}


double Spline::getLenFromT(double t) {
	// Check if the arc lengths lookup table is empty
	if (arcLengths.empty()) {
		animTcl::OutputMessage("Error: Arc lengths lookup table is not initialized.");
		return 0.0;
	}

	t = std::max(0.0, std::min(1.0, t));
	if (t == 1.0) {
		return arcLengths.back().arcLength;
	}
	else if (t == 0.0) {
		return 0.0;
	}

	int i = (t * arcLengths.size()); // u/distance between entries (1/n)
	// This is to convert my stored t (which is n/sample points) into the parametrized t (n/340)
	i = std::min(i, static_cast<int>(arcLengths.size()) - 2);
	double cur = static_cast<double>(i) / static_cast<double>(arcLengths.size());

	if (i == arcLengths.size()) {
		return arcLengths.back().arcLength;
	}

	// S[i] + ((u - U[i])/(U[i+1] - U[i])) * (S[i+1] - S[i]
	double arcLengthStart = arcLengths[i].arcLength; //S[i]
	double arcLengthEnd = arcLengths[i + 1].arcLength; // S[i+1

	double paramStart = (arcLengths[i].t / numPoints); // U[i] 
	double paramEnd = (arcLengths[i + 1].t / numPoints); // U[i+1]

	double paramRange = paramEnd - paramStart; // U[i+1] - U[i]
	double normalizedParam = std::max(0.0, std::min(1.0, (t - paramStart) / paramRange)); // ((u - U[i])/(U[i+1] - U[i])) 	
	// Linear interpolation between arc lengths at  i and i+1
	double val = arcLengthStart + normalizedParam * (arcLengthEnd - arcLengthStart);

	return val;
}

double Spline::getTfromSecant(double len) {
	if (arcLengths.size() == 0) {
		return 0.0;
	}

	int maxIterations = 20;
	double tolerance = 1e-6;
	double t0 = 0.0;
	double t1 = 1.0;

	// Perform the secant method iteration
	for (int iteration = 0; iteration < maxIterations; ++iteration) {
		double len0 = getLenFromT(t0);
		double len1 = getLenFromT(t1);

		// Check for division by zero
		if (len1 - len0 == 0.0) {
			return t1;
		}

		// Compare with len 
		double nextT = t1 - ((len1 - len) * (t1 - t0)) / (len1 - len0);

		// Check for convergence based on arc length
		//if (std::abs(getLenFromT(nextT) - len) < tolerance) {
		if (std::abs(nextT - t1) < tolerance) {
			return nextT;
		}

		// Update values for the next iteration
		t0 = t1;
		t1 = nextT;
	}

	animTcl::OutputMessage("Error: Secant method did not converge within the specified number of iterations.");
	return 0.0;
}

int Spline::load(const std::string& filename)
{
	std::ifstream file(filename);
	if (!file.is_open())
	{
		animTcl::OutputMessage("Error: Unable to open file %s for reading.", filename.c_str());
		return TCL_ERROR;
	}

	// Clear existing points before loading
	points.clear();

	std::string splineName;
	int nPoints;
	file >> splineName >> nPoints;
	if (nPoints > maxPoints) {
		animTcl::OutputMessage("Exceeded man number of points 40");
		return TCL_ERROR;
	}

	for (int i = 0; i < nPoints; ++i)
	{
		glm::dvec3 new_pos, new_tan;
		file >> new_pos.x >> new_pos.y >> new_pos.z >> new_tan.x >> new_tan.y >> new_tan.z;
		addPoint(new_pos, new_tan);
	}

	file.close();
	animTcl::OutputMessage("Spline loaded from %s.", filename.c_str());

	initHermite();

	return TCL_OK;
}

glm::dvec3 Spline::getCarPosition(double& velocity, double timeStep, double& distance) {
	//distance += velocity * timeStep;
	double t = getTfromSecant(distance);
	t = std::max(0.0, std::min(t, 1.0));

	if (t < 0.1) {

		velocity = easeIn(t);

		//distance += velocity * timeStep;

		// Recalculate t based on the new distance
		t = getTfromSecant(distance);
	}
	else if (t > 0.9) {
		velocity = easeOut(t);

		//distance += velocity * timeStep;

		// Recalculate t based on the new distance
		t = getTfromSecant(distance);
	}

	int i = t * numPoints;
	i = std::max(0, std::min(i, static_cast<int>(numPoints - 1)));
	ControlPoint p0 = points[i];
	ControlPoint p1 = points[i + 1];
	double t0 = std::min((t - (static_cast<float>(i) / numPoints)) * numPoints, 1.0);

	glm::dvec3 carPosition = glm::dvec3(evaluateCurve(0, t0, p0, p1),
		evaluateCurve(1, t0, p0, p1),
		evaluateCurve(2, t0, p0, p1));

	//animTcl::OutputMessage("Pos: x: %.3f, y: %.3f, t: %.3f", carPosition[0], carPosition[1], t0);
	return carPosition;
}

glm::dvec3 Spline::getTangents(double velocity, double timeStep, double& distance) {
	distance += velocity * timeStep;
	double t = getTfromSecant(distance);
	t = std::max(0.0, std::min(t, 1.0));
	int i = t * numPoints;
	i = std::max(0, std::min(i, static_cast<int>(numPoints - 1)));

	ControlPoint p0 = points[i];
	ControlPoint p1 = points[i + 1];
	glm::dvec3 tan0, tan1, lerpTan;
	p0.getTan(tan0);
	p1.getTan(tan1);
	//tan0 = getCarPosition(distance);
	//p1.getPos(tan1);
	double t0 = (t - (static_cast<float>(i) / numPoints)) * numPoints;
	lerpTan = glm::dvec3(
		lerp(t0, tan0[0], tan1[0]),
		lerp(t0, tan0[1], tan1[1]),
		lerp(t0, tan0[2], tan1[2]));

	return lerpTan;
}

double Spline::easeIn(double t) {
	double param = t * 10.0;
	param = std::max(0.3, std::min(1.0, param));
	return 1 * ((std::sin((M_PI * param) - M_PI / 2.0) + 1.0) / 2.0);
}

double Spline::easeOut(double t) {
	double param = (t - 0.9) * 10.0;
	param = std::max(0.9, std::min(1.0, param));
	return 1 * ((std::sin((M_PI * param) - M_PI) + 1.0) / 2.0);
}

double Spline::lerp(double t, double p0, double p1) {
	return ((1 - t) * p0) + (t * p1);
}

double Spline::convertLocalTtoGlobalT(double localT) {
	// Convert the local t value to the global parameterized t value
	return localT / static_cast<double>(numSamples);
}

double Spline::convertGlobalTtoLocalT(double globalT) {
	// Convert the global parameterized t value to the local t value
	// Ensure the value is within the valid range [0, 1]
	globalT = std::max(0.0, std::min(1.0, globalT));

	// Map the global parameterized t value to the local t value
	return static_cast<double>(globalT * static_cast<double>(numSamples));
}

int Spline::command(int argc, myCONST_SPEC char** argv)
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
			glmFacetNormals(&m_model);
			glmVertexNormals(&m_model, 90);
			return TCL_OK;
		}
		else
		{
			animTcl::OutputMessage("Usage: read <file_name>");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "cr") == 0)
	{
		if (argc == 1) {
			catMullRom();
			animTcl::OutputMessage("Initializing Catmull-Rom");
			return TCL_OK;
		}
		else {
			animTcl::OutputMessage("Usage: <name> cr");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "getArcLength") == 0) {
		if (argc == 2) {
			try {
				double param = std::stod(argv[1]);
				if (param < 0.0 || param > 1.0) {
					animTcl::OutputMessage("Error: Parameter t should be in the range [0, 1].");
					return TCL_ERROR;
				}

				//double arcLength = getSplineLength(param);
				double arcLength = getLenFromT(param);
				animTcl::OutputMessage("Arc Length at t %.3f: %.3f", param, arcLength);
				return TCL_OK;
			}
			catch (const std::invalid_argument& e) {
				animTcl::OutputMessage("Error: Invalid argument. Please provide a valid number.");
				return TCL_ERROR;
			}
			catch (const std::out_of_range& e) {
				animTcl::OutputMessage("Error: Argument out of range. Please provide a valid number.");
				return TCL_ERROR;

			}
		}
		else {
			animTcl::OutputMessage("Usage: <name> getArcLength <t>");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "getT") == 0) {
		if (argc == 2) {
			try {
				double len = std::stod(argv[1]);
				//double arcLength = getSplineLength(param);
				double t = getTfromSecant(len);
				animTcl::OutputMessage("t at arcLen: %.3f: %.3f", len, t);
				return TCL_OK;
			}
			catch (const std::invalid_argument& e) {
				animTcl::OutputMessage("Error: Invalid argument. Please provide a valid number.");
				return TCL_ERROR;
			}
			catch (const std::out_of_range& e) {
				animTcl::OutputMessage("Error: Argument out of range. Please provide a valid number.");
				return TCL_ERROR;

			}
		}
		else {
			animTcl::OutputMessage("Usage: <name> getArcLength <t>");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "set") == 0 && strcmp(argv[1], "point") == 0)
	{
		if (argc == 6)
		{
			glm::dvec3 new_pos;
			int index = atoi(argv[2]);
			if (index >= points.size())
			{
				animTcl::OutputMessage(" Error: Index out of range");
				return TCL_ERROR;
			}
			new_pos = glm::dvec3(atof(argv[3]), atof(argv[4]), atof(argv[5]));
			points[index].setPos(new_pos);
			//Redraw
			initHermite();
		}
		else
		{
			animTcl::OutputMessage("Usage: <name> set point <index> <x y z>");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "set") == 0 && strcmp(argv[1], "tangent") == 0)
	{
		if (argc == 6)
		{
			glm::dvec3 new_tan;
			int index = atoi(argv[2]);
			if (index >= points.size())
			{
				animTcl::OutputMessage(" Error: Index out of range");
				return TCL_ERROR;
			}
			new_tan = glm::dvec3(atof(argv[3]), atof(argv[4]), atof(argv[5]));
			points[index].setTan(new_tan);
			//Redraw
			initHermite();
		}
		else
		{
			animTcl::OutputMessage("Usage: <name> set point <index> <x y z>");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "add") == 0 && strcmp(argv[1], "point") == 0)
	{
		// Check if the correct number of arguments is provided
		if (argc != 8)
		{
			animTcl::OutputMessage("Usage: <name> add point <n> <p1[x] p1[y] p1[z]");
			return TCL_ERROR;
		}

		glm::dvec3 new_pos, new_tan;

		// Calculate the index for the current point
		int index = points.size();

		// Extract coordinates for the control point
		new_pos = glm::dvec3(atof(argv[2]), atof(argv[3]), atof(argv[4]));

		// Extract coordinates for the tangent
		new_tan = glm::dvec3(atof(argv[5]), atof(argv[6]), atof(argv[7]));

		// Add the point to the spline
		addPoint(new_pos, new_tan);

		//Redraw
		initHermite();

		// Output a success message
		animTcl::OutputMessage("Added point");
	}
	else if (strcmp(argv[0], "load") == 0)
	{
		if (argc == 2)
		{
			return load(argv[1]);
		}
		else
		{
			animTcl::OutputMessage("Usage: system <name> load \"<file name>\"");
			return TCL_ERROR;
		}
	}
	else if (strcmp(argv[0], "flipNormals") == 0)
	{
		flipNormals();
		return TCL_OK;
	}
	else if (strcmp(argv[0], "reset") == 0)
	{
		reset(0);
	}

	glutPostRedisplay();
	return TCL_OK;
}

void Spline::displaySampledCurve(float r) {
	glLineWidth(r);
	glm::dvec3 samplePoints[20];
	glBegin(GL_LINE_STRIP);

	for (int i = 0; i < points.size(); i++) {
		points[i].getPoints(samplePoints);
		for (int j = 0; j < 20; j++)
			glVertex3f(samplePoints[j][0], samplePoints[j][1], samplePoints[j][2]);
	}

	if (points.size() > 0) {
		glm::dvec3 lastPoint;
		points.back().getPos(lastPoint);
		glVertex3f(lastPoint[0], lastPoint[1], lastPoint[2]);
	}
	glEnd();
}

void Spline::displayPoints(float r) {
	// Render each control point as a dot
	glPointSize(r);
	for (size_t i = 0; i < points.size(); ++i)
	{
		glColor3f(1.0f, 0.0f, 0.0f);  // Set dot color
		glBegin(GL_POINTS);
		glm::dvec3 pos, tan;
		points[i].getPos(pos);
		points[i].getTan(tan);
		glVertex3f(pos.x, pos.y, pos.z);
		glEnd();

		// Draw Tangents
		//glColor3f(0.153, 0.227, 0.522);  // Set line color
		glColor3f(0.208, 0.373, 0.38);  // Set line color
		glBegin(GL_LINE_STRIP);
		double tangentLength = 0.5; // You can adjust this length
		glVertex3f(pos.x, pos.y, pos.z);
		glVertex3f(pos.x + tan.x * tangentLength, pos.y + tan.y * tangentLength, pos.z + tan.z * tangentLength);
		glEnd();

	}
}


void Spline::display(GLenum mode)
{
	glEnable(GL_LIGHTING);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glEnable(GL_COLOR_MATERIAL);

	glTranslated(0, 0, 0);

	displayPoints(5.0f);

	glColor3f(0.3, 0.7, 0.1);
	displaySampledCurve(1.5);

	//if (m_model.numvertices > 0)
	//	glmDraw(&m_model, GLM_SMOOTH | GLM_MATERIAL);
	//else
		//glutSolidSphere(1.0, 20, 20);

	glPopMatrix();
	glPopAttrib();
}
