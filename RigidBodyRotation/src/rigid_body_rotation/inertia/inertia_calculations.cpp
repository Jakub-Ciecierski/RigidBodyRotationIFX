#include "rigid_body_rotation/inertia/inertia_calculations.h"

#include <object/render_object.h>
#include <rigid_body_rotation/rigid_body_simulation.h>
#include <math/print_math.h>

double ComputeMass(Cube& cube){
    return cube.density *
            cube.dimensions.x *
            cube.dimensions.y *
            cube.dimensions.z;
}

glm::vec3 ComputeCenterOfMass(Cube& cube){
    glm::vec3 center_of_mass;
    center_of_mass.x = cube.dimensions.x / 2.0;
    center_of_mass.y = cube.dimensions.y / 2.0;
    center_of_mass.z = cube.dimensions.z / 2.0;
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

EulerOutput EulerEquation(const glm::quat& Q0, const glm::vec3& W0,
                          const EulerInputConst& in_const){
    return EulerOutput{W1(Q0, W0, in_const),
                       Q1(Q0, W0, in_const)};
}

glm::vec3 W1(const glm::quat& Q0, const glm::vec3& W0,
             const EulerInputConst& in_const){
    auto h = in_const.dt;

    auto k1 = Wt(Q0, W0, in_const);
    auto k2 = Wt(Q0, W0 + (k1 * h / 2.0f), in_const);
    auto k3 = Wt(Q0, W0 + (k2 * h / 2.0f), in_const);
    auto k4 = Wt(Q0, W0 + (k3 * h), in_const);

    auto m = (k1 + (k2*2.0f) + (k3*2.0f) + k4) * (1.0f/6.0f);
    auto W1 = W0 + (h * m);

    return W1;
}

glm::quat Q1(const glm::quat& Q0, const glm::vec3& W0,
             const EulerInputConst& in_const){
    auto h = in_const.dt;

    auto k1 = Qt(Q0, W0);
    auto k2 = Qt(Q0 + (k1 * h / 2.0f), W0);
    auto k3 = Qt(Q0 + (k2 * h / 2.0f), W0);
    auto k4 = Qt(Q0 + (k3 * h), W0);

    auto m = (k1 + (k2*2.0f) + (k3*2.0f) + k4) * (1.0f/6.0f);
    auto Q1 = Q0 + (h * m);

    return Q1;
}

glm::vec3 Wt(const glm::quat& Q, const glm::vec3& W,
             const EulerInputConst& in_const){
    //auto Q_norm = glm::normalize(Q);
    auto Q_norm = Q;

    auto I_W = in_const.inertia_tensor * W;
    auto a = glm::cross(I_W, W);
    auto N = Torque(Q_norm, in_const);

    return in_const.inertia_tensor_inv * (N + a);
}

glm::quat Qt(const glm::quat& Q, const glm::vec3& W){
    //auto Q_norm = glm::normalize(Q);
    auto Q_norm = Q;
    auto Qw = glm::quat(0, W.x, W.y, W.z);

    return Q_norm * Qw * 0.5f;
}

glm::vec3 Torque(const glm::quat& Q,
                 const EulerInputConst& in_const){
    auto C = in_const.center;
    auto f = in_const.force;
    auto F = glm::conjugate(Q)*f;

    return glm::cross(C, F);
}