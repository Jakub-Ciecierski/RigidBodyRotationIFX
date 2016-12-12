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
};

struct EulerInputConst{
    glm::vec3 center;
    glm::vec3 force;
    glm::mat3 inertia_tensor;
    float dt;
};

EulerOutput EulerEquation(const glm::quat& Q0, const glm::vec3& W0,
                          const EulerInputConst& in_const);

glm::vec3 Wt(const glm::quat& Q, const glm::quat& Qt,
             const EulerInputConst& in_const);

glm::quat Qt(const glm::quat& Q, const glm::vec3& W);

glm::quat Qtt(const glm::quat& Q, const glm::quat& Qt,
              const EulerInputConst& in_const);

glm::vec3 torque(const glm::quat& Q,
                 const EulerInputConst& in_const);

#endif //PROJECT_INERTIA_CALCULATIONS_H
