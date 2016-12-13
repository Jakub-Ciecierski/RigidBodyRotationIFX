#ifndef PROJECT_INERTIA_CALCULATIONS_H
#define PROJECT_INERTIA_CALCULATIONS_H

#include <math/math_ifx.h>

/**
 * http://www.mare.ee/indrek/varphi/vardyn.pdf
 *
 */

struct Cube;

double ComputeMass(Cube& cube);
glm::highp_dvec3 ComputeCenterOfMass(Cube& cube);
glm::highp_dmat3 ComputeIntertiaTensorAroundOrigin(Cube& cube);

struct EulerOutput{
    glm::highp_dvec3 angular_velocity;
    glm::highp_dquat quaternion;
    glm::highp_dquat qt;
};

struct EulerInputConst{
    glm::highp_dvec3 center;
    glm::highp_dvec3 force;
    glm::highp_dmat3 inertia_tensor;
    glm::highp_dmat3 inertia_tensor_inv;
    double dt;
};

EulerOutput EulerEquation(const glm::highp_dquat& Q0, const glm::highp_dvec3& W0,
                          const EulerInputConst& in_const);

glm::highp_dvec3 W1(const glm::highp_dquat& Q0, const glm::highp_dvec3& W0,
                    const EulerInputConst& in_const);
glm::highp_dquat Q1(const glm::highp_dquat& Q0, const glm::highp_dvec3& W0,
                    const EulerInputConst& in_const);

glm::highp_dvec3 Wt(const glm::highp_dquat& Q, const glm::highp_dvec3& W,
                    const EulerInputConst& in_const);
glm::highp_dquat Qt(const glm::highp_dquat& Q, const glm::highp_dvec3& W);

glm::highp_dvec3 Torque(const glm::highp_dquat& Q,
                        const EulerInputConst& in_const);

#endif //PROJECT_INERTIA_CALCULATIONS_H
