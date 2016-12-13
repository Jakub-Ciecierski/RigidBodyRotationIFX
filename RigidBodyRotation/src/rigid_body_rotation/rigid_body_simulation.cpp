#include "rigid_body_rotation/rigid_body_simulation.h"

#include <rigid_body_rotation/inertia/inertia_calculations.h>
#include <rigid_body_rotation/scene_factory.h>
#include <object/render_object.h>
#include <GLFW/glfw3.h>
#include <math/print_math.h>

RigidBodySimulation::RigidBodySimulation(
        SceneFactoryObjects& scene_factory_objects){
    SetSceneObjects(scene_factory_objects);

    auto cube = GetDefaultCube();
    Reset(cube);

    Pause();
}

RigidBodySimulation::~RigidBodySimulation(){}

void RigidBodySimulation::Reset(Cube& cube){
    cube_ = cube;
    ResetTimeData();
    ResetInertiaTensorData();
    UpdateViewObjects();
}

void RigidBodySimulation::Update(){
    time_data_.current_time = glfwGetTime();
    if(!running_){
        time_data_.last_time = time_data_.current_time;
        return;
    }
    double elapsed = time_data_.current_time - time_data_.last_time;
    time_data_.time_since_last_update += elapsed;
    time_data_.total_time += elapsed;
    time_data_.last_time = time_data_.current_time;

    if(time_data_.time_since_last_update >= time_data_.time_delta){
        time_data_.time_since_last_update = 0.0f;
        Update(elapsed);
    }
}

void RigidBodySimulation::Update(double elapsed){
    EulerInputConst euler_input_const{
            inertia_data_.center_body,
            inertia_data_.force,
            inertia_data_.inertia_tensor_body,
            inertia_data_.inertia_tensor_inv_body,
            time_data_.time_delta
    };
    auto euler_output = EulerEquation(inertia_data_.quat_rotation_current,
                                      inertia_data_.angular_velocity_current,
                                      euler_input_const);
    inertia_data_.angular_velocity_current
            = euler_output.angular_velocity;
    inertia_data_.quat_rotation_current
            = glm::normalize(euler_output.quaternion);

    UpdateViewObjects();
}

void RigidBodySimulation::UpdateViewObjects(){
    views_.cube->scale(cube_.dimensions);
    views_.diagonal->scale(glm::vec3(1, cube_.GetDiagonalLength(), 1));

    views_.diagonal->rotateTo(inertia_data_.GetDiagonalRotation());
    views_.cube->rotateTo(inertia_data_.GetCubeRotation());
}

void RigidBodySimulation::ResetTimeData(){
    time_data_.total_time = 0.0f;
    time_data_.current_time = 0.0f;
    time_data_.time_since_last_update = 0.0f;
    time_data_.last_time = glfwGetTime();
}

void RigidBodySimulation::ResetInertiaTensorData(){
    glm::vec3 center = ComputeCenterOfMass(cube_);
    glm::mat3 tensor = ComputeIntertiaTensorAroundOrigin(cube_);

    inertia_data_.diagonal_body = GetDiagonalVector();
    inertia_data_.center_body = GetRotationQuatInitial() * center;

    inertia_data_.inertia_tensor_body
            = GetRotationMatrixInitial()
              * tensor * glm::transpose(GetRotationMatrixInitial());
    inertia_data_.inertia_tensor_inv_body
            = glm::inverse(inertia_data_.inertia_tensor_body);
    inertia_data_.force
            = glm::vec3(0, -ComputeMass(cube_) * cube_.gravity_force, 0);

    inertia_data_.quat_rotation_current
            = glm::normalize(glm::quat(glm::vec3(
            0, 0, glm::radians(cube_.diagonal_rotation_initial))));

    inertia_data_.angular_velocity_current
            = inertia_data_.diagonal_body * cube_.angular_velocity_initial;

    std::cout << "Tensor (Rotated): " << std::endl;
    ifx::PrintMat3(inertia_data_.inertia_tensor_body);
    std::cout << std::endl;

    std::cout << "Center (Rotated): " << std::endl;
    ifx::PrintVec3(inertia_data_.center_body);
    std::cout << std::endl;

    std::cout << "Angular Velocity " << std::endl;
    ifx::PrintVec3(inertia_data_.angular_velocity_current);
    std::cout << std::endl;
}

void RigidBodySimulation::SetSceneObjects(
        SceneFactoryObjects& scene_factory_objects){
    views_.cube = scene_factory_objects.cube;
    views_.diagonal = scene_factory_objects.diagonal;
    views_.force = scene_factory_objects.gravity_vector;
}

Cube RigidBodySimulation::GetDefaultCube(){
    Cube cube;

    cube.dimensions = glm::vec3(1,1,1);
    cube.density = 1.0;
    cube.angular_velocity_initial = 0;
    cube.diagonal_rotation_initial = 0;
    cube.gravity_force = 9.81;
    cube.trajectory_display_count = 1000;

    return cube;
}
