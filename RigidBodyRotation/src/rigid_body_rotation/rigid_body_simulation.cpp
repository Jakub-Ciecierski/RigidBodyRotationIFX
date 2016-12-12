#include "rigid_body_rotation/rigid_body_simulation.h"

#include <rigid_body_rotation/inertia/inertia_calculations.h>
#include <rigid_body_rotation/scene_factory.h>
#include <object/render_object.h>
#include <GLFW/glfw3.h>
#include <math/print_math.h>

RigidBodySimulation::RigidBodySimulation(
        SceneFactoryObjects& scene_factory_objects) :
        grivity_on_(true){
    SetSceneObjects(scene_factory_objects);
    auto params = GetDefaultParameters();
    Reset(params);
    Pause();
}

RigidBodySimulation::~RigidBodySimulation(){}

void RigidBodySimulation::Reset(
        std::shared_ptr<RigidBodySimulationCreateParams> create_params){
    ResetTimeData();
    ResetCubeData(create_params);
    ResetInertiaTensorData();
    ResetViewObjects();
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
            time_data_.time_delta
    };
    auto euler_output = EulerEquation(inertia_data_.quat_rotation_current,
                                      inertia_data_.angular_velocity_current,
                                      euler_input_const);
    inertia_data_.angular_velocity_current
            = euler_output.angular_velocity;
    inertia_data_.quat_rotation_current
            = glm::normalize(euler_output.quaternion);

    /*
    std::cout << "Velocity: " << std::endl;
    ifx::PrintVec3(inertia_data_.angular_velocity_current);
    std::cout << "Quat: " << std::endl;
    ifx::PrintQuat(inertia_data_.quat_rotation_current);*/
/*
    cube_.diagonal_render_object->rotateTo(
            glm::degrees(glm::eulerAngles(
                    inertia_data_.quat_rotation_current)));*/
    /*
    cube_.render_object->rotateTo(
            cube_.GetRotationAngles() +
            glm::degrees(glm::eulerAngles(
                    inertia_data_.quat_rotation_current)));
*/
    cube_.diagonal_render_object->rotateTo(
            glm::degrees(glm::eulerAngles(
                    inertia_data_.quat_rotation_current)));
    cube_.render_object->rotateTo(
            glm::degrees(
                    glm::eulerAngles(inertia_data_.quat_rotation_current)));
}

void RigidBodySimulation::SetSceneObjects(
        SceneFactoryObjects& scene_factory_objects){
    cube_.render_object = scene_factory_objects.cube;
    cube_.diagonal_render_object = scene_factory_objects.diagonal;
    cube_.force_render_object = scene_factory_objects.gravity_vector;
}

std::shared_ptr<RigidBodySimulationCreateParams>
RigidBodySimulation::GetDefaultParameters(){
    auto params = std::shared_ptr<RigidBodySimulationCreateParams>(
            new RigidBodySimulationCreateParams());
    params->dimensions = glm::vec3(1,1,1);
    params->density = 1.0;
    params->angular_velocity = 0;
    params->diagonal_rotation = 0;
    params->gravity_force = 9.81;
    params->gravity_on = true;

    return params;
}

void RigidBodySimulation::ResetTimeData(){
    time_data_.total_time = 0.0f;
    time_data_.current_time = 0.0f;
    time_data_.time_since_last_update = 0.0f;
    time_data_.last_time = glfwGetTime();
}

void RigidBodySimulation::ResetCubeData(
        std::shared_ptr<RigidBodySimulationCreateParams> create_params){
    cube_.dimensions = create_params->dimensions;
    cube_.density = create_params->density;
    cube_.angular_velocity_initial = create_params->angular_velocity;
    cube_.diagonal_rotation_initial = create_params->diagonal_rotation;

    cube_.gravity_force = create_params->gravity_force;
    cube_.gravity_on = create_params->gravity_on;
}

void RigidBodySimulation::ResetInertiaTensorData(){
    glm::vec3 center = ComputeCenterOfMass(cube_);
    glm::mat3 tensor = ComputeIntertiaTensorAroundOrigin(cube_);

    inertia_data_.change_of_basis_matrix = cube_.GetRotationMatrix();
    inertia_data_.diagonal_body = cube_.GetDiagonalVector();
    inertia_data_.center_body = inertia_data_.change_of_basis_matrix * center;
    inertia_data_.inertia_tensor_body
            = inertia_data_.change_of_basis_matrix *
              tensor * glm::transpose(inertia_data_.change_of_basis_matrix);
    inertia_data_.force
            = glm::vec3(0, -ComputeMass(cube_) * cube_.gravity_force, 0);

    inertia_data_.quat_rotation_current = cube_.GetRotationQuat();
    inertia_data_.quat_rotation_current
            = glm::quat(glm::vec3(0, 0,
                                glm::radians(cube_.diagonal_rotation_initial)))
            * inertia_data_.quat_rotation_current;
    inertia_data_.quat_rotation_current
            = glm::normalize(inertia_data_.quat_rotation_current);

    inertia_data_.angular_velocity_current
            = inertia_data_.diagonal_body
              * inertia_data_.angular_velocity_current;

    //ifx::PrintQuat(inertia_data_.quat_rotation_current);
}

void RigidBodySimulation::ResetViewObjects(){
    cube_.render_object->scale(cube_.dimensions);
    cube_.diagonal_render_object->scale(
            glm::vec3(1, cube_.dimensions.x*sqrt(3),1));

    cube_.diagonal_render_object->
            rotateTo(glm::degrees(glm::eulerAngles(
            glm::quat(
                    glm::vec3(
                            0, 0,
                            glm::radians(cube_.diagonal_rotation_initial))))));

    cube_.render_object->rotateTo(
            glm::degrees(
                    glm::eulerAngles(inertia_data_.quat_rotation_current)));
}