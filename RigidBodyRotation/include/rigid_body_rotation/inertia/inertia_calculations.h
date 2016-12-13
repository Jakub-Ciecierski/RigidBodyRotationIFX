#ifndef PROJECT_INERTIA_CALCULATIONS_H
#define PROJECT_INERTIA_CALCULATIONS_H

#include <math/math_ifx.h>

/**
 * http://www.mare.ee/indrek/varphi/vardyn.pdf
 *
 */

struct Cube;

double ComputeMass(Cube& cube);
glm::vec3 ComputeCenterOfMass(Cube& cube);
glm::mat3 ComputeIntertiaTensorAroundOrigin(Cube& cube);

struct EulerOutput{
    glm::vec3 angular_velocity;
    glm::quat quaternion;
    glm::quat qt;
};

struct EulerInputConst{
    glm::vec3 center;
    glm::vec3 force;
    glm::mat3 inertia_tensor;
    glm::mat3 inertia_tensor_inv;
    float dt;
};

EulerOutput EulerEquation(const glm::quat& Q0, const glm::vec3& W0,
                          const EulerInputConst& in_const);

glm::vec3 W1(const glm::quat& Q0, const glm::vec3& W0,
             const EulerInputConst& in_const);
glm::quat Q1(const glm::quat& Q0, const glm::vec3& W0,
             const EulerInputConst& in_const);

glm::vec3 Wt(const glm::quat& Q, const glm::vec3& W,
             const EulerInputConst& in_const);
glm::quat Qt(const glm::quat& Q, const glm::vec3& W);

glm::vec3 Torque(const glm::quat& Q,
                 const EulerInputConst& in_const);

#endif //PROJECT_INERTIA_CALCULATIONS_H
