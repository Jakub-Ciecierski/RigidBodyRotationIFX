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

glm::highp_dvec3 ComputeCenterOfMass(Cube& cube){
    glm::highp_dvec3 center_of_mass;
    center_of_mass.x = cube.dimensions.x / 2.0;
    center_of_mass.y = cube.dimensions.y / 2.0;
    center_of_mass.z = cube.dimensions.z / 2.0;
    center_of_mass = center_of_mass * cube.density;

    return center_of_mass;
}

glm::highp_dmat3 ComputeIntertiaTensorAroundOrigin(Cube& cube){
    glm::highp_dmat3 inertia_tensor;
    double a = cube.dimensions.x;
    double b = cube.dimensions.y;
    double c = cube.dimensions.z;

    double abc = a*b*c;
    double a_sqr = a*a;
    double b_sqr = b*b;
    double c_sqr = c*c;

    double xy = -0.25f * a_sqr * b_sqr * c;
    double xz = -0.25 * a_sqr * b * c_sqr;
    double yz = -0.25 * a * b_sqr * c_sqr;

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

EulerOutput EulerEquation(const glm::highp_dquat& Q0, const glm::highp_dvec3& W0,
                          const EulerInputConst& in_const){
    return EulerOutput{W1(Q0, W0, in_const),
                       Q1(Q0, W0, in_const)};
}

glm::highp_dvec3 W1(const glm::highp_dquat& Q0, const glm::highp_dvec3& W0,
             const EulerInputConst& in_const){
    auto h = in_const.dt;

    auto k1 = Wt(Q0, W0, in_const);
    auto k2 = Wt(Q0, W0 + (k1 * h / 2.0), in_const);
    auto k3 = Wt(Q0, W0 + (k2 * h / 2.0), in_const);
    auto k4 = Wt(Q0, W0 + (k3 * h), in_const);

    auto m = (k1 + (k2*2.0) + (k3*2.0) + k4) * (1.0/6.0);
    auto W1 = W0 + (h * m);

    return W1;
}

glm::highp_dquat Q1(const glm::highp_dquat& Q0, const glm::highp_dvec3& W0,
             const EulerInputConst& in_const){
    auto h = in_const.dt;

    auto k1 = Qt(Q0, W0);
    auto k2 = Qt(Q0 + (k1 * h / 2.0), W0);
    auto k3 = Qt(Q0 + (k2 * h / 2.0), W0);
    auto k4 = Qt(Q0 + (k3 * h), W0);

    auto m = (k1 + (k2*2.0) + (k3*2.0) + k4) * (1.0/6.0);
    auto Q1 = Q0 + (h * m);

    return Q1;
}

glm::highp_dvec3 Wt(const glm::highp_dquat& Q, const glm::highp_dvec3& W,
             const EulerInputConst& in_const){
    //auto Q_norm = glm::normalize(Q);
    auto Q_norm = Q;

    auto I_W = in_const.inertia_tensor * W;
    auto a = glm::cross(I_W, W);
    auto N = Torque(Q_norm, in_const);

    return in_const.inertia_tensor_inv * (N + a);
}

glm::highp_dquat Qt(const glm::highp_dquat& Q, const glm::highp_dvec3& W){
    //auto Q_norm = glm::normalize(Q);
    auto Q_norm = Q;
    auto Qw = glm::highp_dquat(0, W.x, W.y, W.z);

    return Q_norm * Qw * 0.5;
}

glm::highp_dvec3 Torque(const glm::highp_dquat& Q,
                 const EulerInputConst& in_const){
    auto C = in_const.center;
    auto f = in_const.force;
    auto F = glm::conjugate(Q)*f;

    return glm::cross(C, F);
}