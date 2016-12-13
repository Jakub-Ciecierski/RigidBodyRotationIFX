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

    cube_ = simulation_->cube();
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
    //simulation_->Reset(simulation_create_params_);
    if (ImGui::Button("Reset")) {
        simulation_->Reset(cube_);
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
    RenderCubeParametersDiagonalRotation();
}

void RigidBodyGUI::RenderCubeParametersDimension(){
    static float dim;
    dim = cube_.dimensions.x;

    ImGui::SliderFloat("Dimensions", &dim, 1, 5);

    cube_.dimensions.x = dim;
    cube_.dimensions.y = dim;
    cube_.dimensions.z = dim;
}

void RigidBodyGUI::RenderCubeParametersDensity(){
    static float c = cube_.density;
    ImGui::SliderFloat("Density",
                       &c, 1, 5);
    cube_.density = c;
}

void RigidBodyGUI::RenderCubeParametersAngularVelocity(){
    static float c = cube_.angular_velocity_initial;

    ImGui::SliderFloat("Angular Velocity (Around diagonal)",
                       &c,
                       0, 20);

    cube_.angular_velocity_initial = c;
}

void RigidBodyGUI::RenderCubeParametersDiagonalRotation(){
    static float c = cube_.diagonal_rotation_initial;
    ImGui::SliderFloat("Diagonal Rotation",
                       &c,
                       0, 360);

    cube_.diagonal_rotation_initial = c;
}

void RigidBodyGUI::RenderTrajectoryParemeters(){
    ImGui::SliderInt("Trajectory length",
                     &cube_.trajectory_display_count, 1, 10000);
}

void RigidBodyGUI::RenderGravityParemeters(){
    static float c = cube_.gravity_force;
    ImGui::SliderFloat("Gravity",
                       &c,
                       0, 20);

    if (ImGui::Button("0.00")) {
        c = 0.0;
    }
    ImGui::SameLine();
    if (ImGui::Button("9.81")) {
        c = 9.81;
    }

    cube_.gravity_force = c;
}