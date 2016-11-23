#include "rigid_body_rotation/inertia/inertia_calculations.h"

#include <object/render_object.h>
#include <rigid_body_rotation/rigid_body_simulation.h>

float ComputeMass(Cube& cube){
    return cube.density *
            cube.dimensions.x *
            cube.dimensions.y *
            cube.dimensions.z;
}

glm::vec3 ComputeCenterOfMass(Cube& cube){
    float mass = ComputeMass(cube);

    glm::vec3 center_of_mass;
    center_of_mass.x = cube.dimensions.x / 2.0f;
    center_of_mass.y = cube.dimensions.y / 2.0f;
    center_of_mass.z = cube.dimensions.z / 2.0f;
    center_of_mass = center_of_mass * cube.density;

    return center_of_mass;
}

glm::mat3 ComputeIntertiaTensorAroundOrigin(Cube& cube){
    glm::mat3 inertia_tensor;
    float a = cube.dimensions.x;
    float b = cube.dimensions.y;
    float c = cube.dimensions.z;

    float abc = a*b*c;
    float a_sqr = a*a;
    float b_sqr = b*b;
    float c_sqr = c*c;

    float xy = -0.25f * a_sqr * b_sqr * c;
    float xz = -0.25 * a_sqr * b * c_sqr;
    float yz = -0.25 * a * b_sqr * c_sqr;

    inertia_tensor[0].x = 0.333f * abc * (b_sqr + c_sqr);
    inertia_tensor[0].y = yz;
    inertia_tensor[0].z = xz;

    inertia_tensor[1].x = xy;
    inertia_tensor[1].y = 0.333f * abc * (a_sqr + c_sqr);
    inertia_tensor[1].z = yz;

    inertia_tensor[2].x = xz;
    inertia_tensor[2].y = yz;
    inertia_tensor[2].z = 0.333f * abc * (a_sqr + b_sqr);

    inertia_tensor = inertia_tensor * cube.density;

    return inertia_tensor;
}