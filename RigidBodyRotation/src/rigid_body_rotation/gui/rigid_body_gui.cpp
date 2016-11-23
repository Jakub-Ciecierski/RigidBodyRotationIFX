#include <rigid_body_rotation/rigid_body_simulation.h>
#include "rigid_body_rotation/gui/rigid_body_gui.h"

#include "engine_gui/engine_gui.h"
#include "engine_gui/factory/engine_gui_factory.h"

#include <rendering/renderer.h>

#include <gui/imgui/imgui.h>

RigidBodyGUI::RigidBodyGUI(GLFWwindow* window,
                           std::shared_ptr<RigidBodySimulation> simulation,
                           std::shared_ptr<ifx::Renderer> renderer) :
        ifx::GUI(window),
        simulation_(simulation){
    engine_gui_ = ifx::EngineGUIFactory().CreateEngineGUI(renderer);

    simulation_create_params_
            = std::shared_ptr<RigidBodySimulationCreateParams>(
            new RigidBodySimulationCreateParams());
    simulation_create_params_->dimensions = simulation->cube()->dimensions;
    simulation_create_params_->density = simulation->cube()->density;
    simulation_create_params_->angular_velocity
            = simulation->cube()->angular_velocity;
}

RigidBodyGUI::~RigidBodyGUI(){

}

void RigidBodyGUI::Render(){
    NewFrame();

    RenderGUI();
    engine_gui_->Render();

    ImGui::Render();
}

void RigidBodyGUI::RenderGUI(){
    ImGui::SetNextWindowSize(ImVec2(350,600));
    ImGui::Begin("Simulation");

    ImGui::PushItemWidth(200);
    if(ImGui::CollapsingHeader("Simulation"))
        RenderSimulationInfo();
    if(ImGui::CollapsingHeader("Cube"))
        RenderCubeParameters();
    if(ImGui::CollapsingHeader("Trajectory"))
        RenderTrajectoryParemeters();
    if(ImGui::CollapsingHeader("Gravity"))
        RenderGravityParemeters();
    ImGui::End();
}

void RigidBodyGUI::RenderSimulationInfo(){
    ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
    ImGui::Text("Time: %.2f [s]", simulation_->total_time());

    if (ImGui::Button("Reset")) {
        simulation_->SetRunning(true);
        simulation_->Reset(simulation_create_params_);
    }
    ImGui::SameLine();

    std::string play_button_text;
    if (simulation_->IsRunning())
        play_button_text = "Pause";
    else
        play_button_text = "Play";

    if (ImGui::Button(play_button_text.c_str())) {
        simulation_->SetRunning(!simulation_->IsRunning());
    }
}

void RigidBodyGUI::RenderCubeParameters(){
    RenderCubeParametersDimension();
    RenderCubeParametersDensity();
    RenderCubeParametersAngularVelocity();
}

void RigidBodyGUI::RenderCubeParametersDimension(){
    static float dim;
    dim = simulation_create_params_->dimensions.x;

    ImGui::SliderFloat("Dimensions", &dim, 1, 5);

    simulation_create_params_->dimensions.x = dim;
    simulation_create_params_->dimensions.y = dim;
    simulation_create_params_->dimensions.z = dim;
}

void RigidBodyGUI::RenderCubeParametersDensity(){
    ImGui::SliderFloat("Density", &simulation_create_params_->density, 1, 5);
}

void RigidBodyGUI::RenderCubeParametersAngularVelocity(){
    static float raw[3];
    raw[0] = simulation_create_params_->angular_velocity.x;
    raw[1] = simulation_create_params_->angular_velocity.y;
    raw[2] = simulation_create_params_->angular_velocity.z;

    ImGui::SliderFloat3("Angular Velocity", raw, 1, 5);

    simulation_create_params_->angular_velocity.x = raw[0];
    simulation_create_params_->angular_velocity.y = raw[1];
    simulation_create_params_->angular_velocity.z = raw[2];
}

void RigidBodyGUI::RenderTrajectoryParemeters(){

}

void RigidBodyGUI::RenderGravityParemeters(){

}