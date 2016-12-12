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
    auto h = in_const.dt;
    auto h2 = h*h;

    auto Qt0 = Qt(Q0, W0);

    auto k1 = Qtt(Q0, Qt0, in_const);
    auto k2 = Qtt(
            Q0 + (Qt0 * (h / 2.0f)) + (k1*(h2/8.0f)),
            Qt0 + (k1 * (h/2.0f)), in_const);
    auto k3 = Qtt(
            Q0 + (Qt0 * (h/2.0f)) + (k2*(h2/8.0f)), // k1
            Qt0 + (k2 * (h/2.0f)), in_const);
    auto k4 = Qtt(
            Q0 + (Qt0 * h) + (k3*(h2/2.0f)),
            Qt0 + (k3 * h), in_const);

    auto Q1 = glm::normalize(Q0 + Qt0*h + ((k1 + k2 + k3) * (h2/6.0f)));
    auto Qt1 = Qt0 + (k1 + k2*2.0f + k3*2.0f + k4) *  (h / 6.0f);

    //auto Q1 = glm::normalize(Qt1 * Q0 );

    auto Q_W1 = glm::inverse(Q1) * Qt1 * 2.0f;
    //Q_W1 = glm::normalize(Q_W1);
    auto W1 = glm::vec3(Q_W1.x, Q_W1.y, Q_W1.z);

    return EulerOutput{W1, Q1};
}

glm::vec3 Wt(const glm::quat& Q, const glm::quat& Qt,
             const EulerInputConst& in_const){
    auto N = torque(Q, in_const);
    auto I = in_const.inertia_tensor;

    auto Qm = glm::inverse(Q)*Qt;
    auto v = glm::vec3(Qm.x, Qm.y, Qm.z);

    auto a = I * v;
    auto b = glm::cross(a, v) * 4.0f;

    return glm::inverse(I) * (N + b);
}

glm::quat Qt(const glm::quat& Q, const glm::vec3& W){
    glm::quat Qw(0, W.x, W.y, W.z);

    return 0.5f * Q * Qw;
}

glm::quat Qtt(const glm::quat& Q, const glm::quat& Qt,
              const EulerInputConst& in_const){
    auto a = Qt * glm::inverse(Q) * Qt;

    auto Wt_v = Wt(Q, Qt, in_const);
    auto Q_Wt = glm::quat(0, Wt_v.x, Wt_v.y, Wt_v.z);

    auto b = Q * Q_Wt * 0.5f;

    return a+b;
}

glm::vec3 torque(const glm::quat& Q,
                 const EulerInputConst& in_const) {
    auto C = in_const.center;
    auto f = in_const.force;
    auto F = glm::inverse(Q) * f;

    return glm::cross(C, F);
}