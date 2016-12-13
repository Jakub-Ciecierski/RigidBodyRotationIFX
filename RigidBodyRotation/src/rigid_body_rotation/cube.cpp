#include <rigid_body_rotation/cube.h>

glm::highp_dvec3 GetRotationAnglesInitial(){
    return glm::highp_dvec3(0.0,
                     glm::degrees(atan(1.0)),
                     glm::degrees(asin(sqrt(2.0)/sqrt(3.0))));
};

// From origin (x,y,z) to starting rotated frame (X,Y,Z).
// Change of basis matrix, where diagonal is aligned with y axis
glm::highp_dmat3 GetRotationMatrixInitial(){
    return glm::mat3_cast(GetRotationQuatInitial());
};

glm::highp_dquat GetRotationQuatInitial(){
    return glm::normalize(glm::highp_dquat(
            glm::radians(GetRotationAnglesInitial())));
}

glm::highp_dvec3 GetDiagonalVector(){
    return glm::highp_dvec3(0.0, 1.0, 0.0);
}
