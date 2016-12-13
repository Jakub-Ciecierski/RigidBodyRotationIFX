#include <rigid_body_rotation/cube.h>

glm::vec3 GetRotationAnglesInitial(){
    return glm::vec3(0,
                     glm::degrees(atan(1L)),
                     glm::degrees(asin(sqrt(2L)/sqrt(3L))));
};

// From origin (x,y,z) to starting rotated frame (X,Y,Z).
// Change of basis matrix, where diagonal is aligned with y axis
glm::mat3 GetRotationMatrixInitial(){
    return glm::mat3_cast(GetRotationQuatInitial());
};

glm::quat GetRotationQuatInitial(){
    return glm::normalize(glm::quat(glm::radians(GetRotationAnglesInitial())));
}

glm::vec3 GetDiagonalVector(){
    return glm::vec3(0L, 1L, 0L);
}
