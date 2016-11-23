#include "rigid_body_rotation/rigid_body_simulation.h"

#include <rigid_body_rotation/inertia/inertia_calculations.h>
#include <rigid_body_rotation/scene_factory.h>
#include <object/render_object.h>
#include <GLFW/glfw3.h>
#include <math/print_math.h>

RigidBodySimulation::RigidBodySimulation(
        SceneFactoryObjects& scene_factory_objects) :
        grivity_on_(true),
        time_delta_(0.001),
        time_delta_sqr_(time_delta_*time_delta_),
        current_update_time_(0),
        last_update_time_(0),
        total_time_s_(0){
    SetSceneObjects(scene_factory_objects);
    auto params = GetDefaultParameters();
    Reset(params);
}

RigidBodySimulation::~RigidBodySimulation(){}

void RigidBodySimulation::Reset(
        std::shared_ptr<RigidBodySimulationCreateParams> create_params){
    cube_.dimensions = create_params->dimensions;
    cube_.density = create_params->density;
    cube_.angular_velocity = create_params->angular_velocity;

    total_time_s_ = 0;

    cube_.render_object->scale(cube_.dimensions);

    PrintTensorInfo();
}

void RigidBodySimulation::Update(){
    if(!SatisfiesTimeDelta())
        return;
}

bool RigidBodySimulation::SatisfiesTimeDelta(){
    current_update_time_ = glfwGetTime();
    if(!running_){
        last_update_time_ = current_update_time_;
        return false;
    }
    double time_delta = current_update_time_ - last_update_time_;
    total_time_s_ += time_delta;

    bool value = time_delta >= time_delta_;
    if(value)
        last_update_time_ = current_update_time_;

    return value;
}

void RigidBodySimulation::SetSceneObjects(
        SceneFactoryObjects& scene_factory_objects){
    cube_.render_object = scene_factory_objects.cube;
}

std::shared_ptr<RigidBodySimulationCreateParams>
RigidBodySimulation::GetDefaultParameters(){
    auto params = std::shared_ptr<RigidBodySimulationCreateParams>(
            new RigidBodySimulationCreateParams());
    params->dimensions = glm::vec3(1,1,1);
    params->density = 1.0f;
    params->angular_velocity = glm::vec3(0,0,0);

    return params;
}

void RigidBodySimulation::PrintTensorInfo(){
    float mass = ComputeMass(cube_);
    glm::vec3 center = ComputeCenterOfMass(cube_);
    glm::mat3 tensor = ComputeIntertiaTensorAroundOrigin(cube_);
    glm::mat3 rotation = cube_.GetRotationMatrix();
    glm::mat3 tensor_rotated = rotation * tensor * glm::transpose(rotation);

    std::cout << "Mass: " << mass << std::endl;
    std::cout << "Center of Mass: " << std::endl;
    ifx::PrintVec3(center);
    std::cout << "Inertia Tensor: " << std::endl;
    ifx::PrintMat3(tensor);
    std::cout << "Inertia Tensor Rotated: " << std::endl;
    ifx::PrintMat3(tensor_rotated);
}