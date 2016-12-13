#ifndef PROJECT_CUBE_H
#define PROJECT_CUBE_H

#include <math/math_ifx.h>

struct Cube {
    glm::highp_dvec3 dimensions;
    double density;
    double angular_velocity_initial;
    double diagonal_rotation_initial;
    double gravity_force;

    int trajectory_display_count;
    bool render_trajectory;

    double GetDiagonalLength(){
        return dimensions.x*sqrt(3.0);
    }
};

glm::highp_dvec3 GetRotationAnglesInitial();

// From origin (x,y,z) to starting rotated frame (X,Y,Z).
// Change of basis matrix, where diagonal is aligned with y axis
glm::highp_dmat3 GetRotationMatrixInitial();

glm::highp_dquat GetRotationQuatInitial();
glm::highp_dvec3 GetDiagonalVector();

#endif //PROJECT_CUBE_H
