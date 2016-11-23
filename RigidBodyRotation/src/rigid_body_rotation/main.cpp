#include <game_loop/game_loop.h>
#include <factory/render_object_factory.h>
#include <rendering/renderer.h>

#include <memory>
#include <rigid_body_rotation/gui/rigid_body_gui.h>
#include <rigid_body_rotation/scene_factory.h>
#include <rigid_body_rotation/rigid_body_simulation.h>

void InitSimulation(ifx::GameLoop& game_loop);
void InitScene(ifx::GameLoop& game_loop,
               SceneFactoryObjects& scene_factory_objects);
void InitGUI(ifx::GameLoop& game_loop,
             std::shared_ptr<RigidBodySimulation> simulation);

void InitSimulation(ifx::GameLoop& game_loop){
    auto scene_factory_objects
            = SceneFactory().CreateSceneObjects();
    auto simulation = std::shared_ptr<RigidBodySimulation>(
            new RigidBodySimulation(scene_factory_objects));

    game_loop.AddSimulation(simulation);

    InitScene(game_loop,scene_factory_objects);
    InitGUI(game_loop, simulation);
}

void InitScene(ifx::GameLoop& game_loop,
               SceneFactoryObjects& scene_factory_objects){
    auto scene = game_loop.renderer()->scene();
    if(scene_factory_objects.cube)
        scene->AddRenderObject(scene_factory_objects.cube);
    if(scene_factory_objects.axis)
        scene->AddRenderObject(scene_factory_objects.axis);
    if(scene_factory_objects.gravity_plane)
        scene->AddRenderObject(scene_factory_objects.gravity_plane);
    if(scene_factory_objects.gravity_vector)
        scene->AddRenderObject(scene_factory_objects.gravity_vector);
}

void InitGUI(ifx::GameLoop& game_loop,
             std::shared_ptr<RigidBodySimulation> simulation){
    auto gui = std::unique_ptr<RigidBodyGUI>(new RigidBodyGUI(
            game_loop.renderer()->window()->getHandle(),
            simulation,
            game_loop.renderer()->scene()));
    game_loop.renderer()->SetGUI(std::move(gui));
}

int main() {
    ifx::GameLoop game_loop(
            std::move(ifx::RenderObjectFactory().CreateRenderer()));

    InitSimulation(game_loop);

    game_loop.Start();
}

