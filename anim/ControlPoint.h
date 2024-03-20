#ifndef CONTROL_POINT_H
#define CONTROL_POINT_H

#include "BaseSystem.h"
#include <shared/defs.h>
#include <util/util.h>
#include "animTcl.h"
#include <vector>
#include <GLmodel/GLmodel.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "shared/opengl.h"

class ControlPoint : public BaseSystem
{
public:
    ControlPoint(const std::string& name);

    // Copy constructor
    ControlPoint(const ControlPoint& other);

    // Copy assignment operator
    ControlPoint& operator=(const ControlPoint& other);

    virtual void getPos(glm::dvec3& pos);
    virtual void setPos(const glm::dvec3& pos);
    virtual void getTan(glm::dvec3& tan);
    virtual void setTan(const glm::dvec3& tan);
    virtual void getPoints(glm::dvec3 points[20]);
    virtual void setSamplePoints(const glm::dvec3 points[20]);
    void reset(double time);

    void display(GLenum mode = GL_RENDER);

    void readModel(const char* fname);
    int command(int argc, myCONST_SPEC char** argv);

protected:
    glm::dvec3 m_pos;
    glm::dvec3 tangent;
    const int numSamples = 20;
    glm::dvec3 samplePoints[20];
    GLMmodel m_model;
};

#endif
