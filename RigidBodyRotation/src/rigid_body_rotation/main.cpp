#include <game/game_loop.h>
#include <factory/render_object_factory.h>
#include <rendering/renderer.h>

#include <memory>
#include <rigid_body_rotation/gui/rigid_body_gui.h>
#include <rigid_body_rotation/scene_factory.h>
#include <rigid_body_rotation/rigid_body_simulation.h>
#include <game/factory/game_loop_factory.h>

void InitSimulation(ifx::GameLoop& game_loop);
void InitScene(ifx::GameLoop& game_loop,
               SceneFactoryObjects& scene_factory_objects);
void InitGUI(ifx::GameLoop& game_loop,
             std::shared_ptr<RigidBodySimulation> simulation);

void InitSimulation(ifx::GameLoop& game_loop){
    auto scene_factory_objects
            = SceneFactory().CreateSceneObjects();
    auto simulation = std::shared_ptr<RigidBodySimulation>(
            new RigidBodySimulation(scene_factory_objects,
                                    game_loop.renderer()->scene()));

    game_loop.AddSimulation(simulation);

    InitScene(game_loop,scene_factory_objects);
    InitGUI(game_loop, simulation);
}

void InitScene(ifx::GameLoop& game_loop,
               SceneFactoryObjects& scene_factory_objects){
    auto scene = game_loop.renderer()->scene();
    scene->camera()->rotateTo(glm::vec3(-118, -10, 0));
    scene->camera()->moveTo(glm::vec3(2.78, 1.77, 4.71));

    auto floor = ifx::RenderObjectFactory().CreateFloor();
    floor->move(glm::vec3(0, -3, 0));
    floor->scale(10);
    scene->AddRenderObject(std::move(floor));

    if(scene_factory_objects.diagonal)
        scene->AddRenderObject(scene_factory_objects.diagonal);
    if(scene_factory_objects.axis)
        scene->AddRenderObject(scene_factory_objects.axis);
    if(scene_factory_objects.gravity_plane)
        scene->AddRenderObject(scene_factory_objects.gravity_plane);
    if(scene_factory_objects.gravity_vector)
        scene->AddRenderObject(scene_factory_objects.gravity_vector);

    if(scene_factory_objects.cube)
        scene->AddRenderObject(scene_factory_objects.cube);
}

void InitGUI(ifx::GameLoop& game_loop,
             std::shared_ptr<RigidBodySimulation> simulation){
    auto gui = std::unique_ptr<RigidBodyGUI>(new RigidBodyGUI(
            game_loop.renderer()->window()->getHandle(),
            simulation,
            game_loop.renderer()));
    game_loop.renderer()->SetGUI(std::move(gui));

    //game_loop.renderer()->LimitFPS(true);
}

int main() {
    auto game_loop = ifx::GameLoopFactory().Create();
    InitSimulation(*game_loop.get());

    game_loop->Start();
}

