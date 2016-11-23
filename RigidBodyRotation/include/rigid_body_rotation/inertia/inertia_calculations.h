#ifndef PROJECT_INERTIA_CALCULATIONS_H
#define PROJECT_INERTIA_CALCULATIONS_H

#include <math/math_ifx.h>

struct Cube;

float ComputeMass(Cube& cube);
glm::vec3 ComputeCenterOfMass(Cube& cube);
glm::mat3 ComputeIntertiaTensorAroundOrigin(Cube& cube);

#endif //PROJECT_INERTIA_CALCULATIONS_H
