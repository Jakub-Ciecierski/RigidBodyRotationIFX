#include "rigid_body_rotation/rigid_body_simulation.h"

#include <rendering/scene/scene.h>
#include <rigid_body_rotation/inertia/inertia_calculations.h>
#include <rigid_body_rotation/scene_factory.h>
#include <object/render_object.h>
#include <GLFW/glfw3.h>
#include <math/print_math.h>
#include <factory/program_factory.h>

RigidBodySimulation::RigidBodySimulation(
        SceneFactoryObjects& scene_factory_objects,
        std::shared_ptr<ifx::Scene> scene) : scene_(scene){
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
        //int iters = 10;
        //for(int i = 0; i < iters; i++)
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
    UpdateTrajectory();
}

void RigidBodySimulation::UpdateViewObjects(){
    views_.cube->scale(cube_.dimensions.x);
    views_.diagonal->scale(glm::vec3(1, cube_.GetDiagonalLength(), 1));

    views_.diagonal->rotateTo((glm::vec3)inertia_data_.GetDiagonalRotation());
    views_.cube->rotateTo((glm::vec3)inertia_data_.GetCubeRotation());
}

void RigidBodySimulation::ResetTimeData(){
    time_data_.total_time = 0.0f;
    time_data_.current_time = 0.0f;
    time_data_.time_since_last_update = 0.0f;
    time_data_.last_time = glfwGetTime();

    trajectory_.clear();
}

void RigidBodySimulation::ResetInertiaTensorData(){
    glm::highp_dvec3 center = ComputeCenterOfMass(cube_);
    glm::highp_dmat3 tensor = ComputeIntertiaTensorAroundOrigin(cube_);

    inertia_data_.diagonal_body = GetDiagonalVector();
    inertia_data_.center_body = GetRotationQuatInitial() * center;

    inertia_data_.inertia_tensor_body
            = GetRotationMatrixInitial()
              * tensor * glm::transpose(GetRotationMatrixInitial());

    inertia_data_.inertia_tensor_body[1].x = 0;
    inertia_data_.inertia_tensor_body[2].x = 0;
    inertia_data_.inertia_tensor_body[0].y = 0;
    inertia_data_.inertia_tensor_body[2].y = 0;
    inertia_data_.inertia_tensor_body[1].z = 0;

    inertia_data_.center_body[0] = 0;
    inertia_data_.center_body[2] = 0;

    inertia_data_.inertia_tensor_inv_body
            = glm::inverse(inertia_data_.inertia_tensor_body);
    inertia_data_.force
            = glm::highp_dvec3(0, -ComputeMass(cube_) * cube_.gravity_force, 0);

    inertia_data_.quat_rotation_current
            = glm::normalize(glm::highp_dquat(glm::highp_dvec3(
            0, 0, glm::radians(cube_.diagonal_rotation_initial))));

    inertia_data_.angular_velocity_current
            = inertia_data_.diagonal_body * cube_.angular_velocity_initial;

    std::cout << "Tensor (Rotated): " << std::endl;
    ifx::PrintMat3((glm::mat3)inertia_data_.inertia_tensor_body);
    std::cout << std::endl;

    std::cout << "Center (Rotated): " << std::endl;
    ifx::PrintVec3((glm::vec3)inertia_data_.center_body);
    std::cout << std::endl;

    std::cout << "Angular Velocity " << std::endl;
    ifx::PrintVec3((glm::vec3)inertia_data_.angular_velocity_current);
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

    cube.dimensions = glm::highp_dvec3(1,1,1);
    cube.density = 1.0;
    cube.angular_velocity_initial = 0;
    cube.diagonal_rotation_initial = 0;
    cube.gravity_force = 9.81;
    cube.trajectory_display_count = 10000;
    cube.render_trajectory = true;

    return cube;
}

void RigidBodySimulation::UpdateTrajectory(){
    if((int)trajectory_.size() > cube_.trajectory_display_count)
        return;

    auto diagonal = inertia_data_.quat_rotation_current * GetDiagonalVector();
    auto pos = diagonal * cube_.GetDiagonalLength();

    trajectory_.push_back((glm::vec3)pos);
    ResetTrajectoryView();
}

void RigidBodySimulation::ResetTrajectoryView(){
    if(views_.trajectory){
        scene_->DeleteRenderObject(views_.trajectory.get());
    }

    std::vector<Vertex> vertices(trajectory_.size());
    std::vector<GLuint> indices(trajectory_.size());
    for(unsigned int i = 0; i < vertices.size(); i++){
        vertices[i] = Vertex{trajectory_[i],
                             glm::vec3(0.0f, 0.0f, -1.0f),
                             glm::vec2(1.0f, 1.0f)};
        indices[i] = i;
    }
    std::unique_ptr<ifx::Mesh> mesh(new ifx::Mesh(vertices, indices));

    auto material = std::make_shared<ifx::Material>();

    material->AddTexture(ifx::Texture2D::MakeTexture2DFromFile(
            ifx::Resources::GetInstance().GetResourcePath(
                    "wood_diffuse.png", ifx::ResourceType::TEXTURE),
            ifx::TextureTypes::DIFFUSE
    ));
    material->AddTexture(ifx::Texture2D::MakeTexture2DFromFile(
            ifx::Resources::GetInstance().GetResourcePath(
                    "wood_specular.png", ifx::ResourceType::TEXTURE),
            ifx::TextureTypes::SPECULAR
    ));

    mesh->material(material);
    mesh->primitive_draw_mode(ifx::PrimitiveDrawMode::LINES);
    std::vector<std::unique_ptr<ifx::Mesh>> meshes;
    meshes.push_back(std::move(mesh));
    auto model = ifx::Model::MakeModel(ifx::NO_FILEPATH, std::move(meshes));

    std::shared_ptr<Program> program = ifx::ProgramFactory().LoadMainProgram();

    views_.trajectory
            = std::shared_ptr<ifx::RenderObject>(new ifx::RenderObject(
            ObjectID(0, "Trajectory"), model));
    views_.trajectory->addProgram(program);
    views_.trajectory->SetBeforeRender(
            [](const Program *program){
                glLineWidth(3);
            }
    );
    views_.trajectory->SetAfterRender(
            [](const Program *program){
                glLineWidth(1);
            }
    );
    views_.trajectory->do_render(cube_.render_trajectory);
    scene_->AddRenderObject(views_.trajectory);
}
