#ifndef PROJECT_CUBE_H
#define PROJECT_CUBE_H

#include <math/math_ifx.h>

struct Cube {
    glm::vec3 dimensions;
    float density;
    float angular_velocity_initial;
    float diagonal_rotation_initial;
    float gravity_force;

    int trajectory_display_count;

    float GetDiagonalLength(){
        return dimensions.x*sqrt(3);
    }
};

glm::vec3 GetRotationAnglesInitial();

// From origin (x,y,z) to starting rotated frame (X,Y,Z).
// Change of basis matrix, where diagonal is aligned with y axis
glm::mat3 GetRotationMatrixInitial();

glm::quat GetRotationQuatInitial();
glm::vec3 GetDiagonalVector();
glm::mat3 Rx(double alpha);
glm::mat3 Ry(double alpha);
glm::mat3 Rz(double alpha);

#endif //PROJECT_CUBE_H
