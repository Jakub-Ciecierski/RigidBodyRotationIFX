#ifndef PROJECT_RIGID_BODY_GUI_H
#define PROJECT_RIGID_BODY_GUI_H

#include <gui/gui.h>

#include <memory>

namespace ifx{
class Scene;
class SceneWindowGUI;
}

struct RigidBodySimulationCreateParams;
class RigidBodySimulation;

class RigidBodyGUI : public ifx::GUI {
public:

    RigidBodyGUI(GLFWwindow* window,
                 std::shared_ptr<RigidBodySimulation> simulation,
                 std::shared_ptr<ifx::Scene> scene);
    ~RigidBodyGUI();

    virtual void Render() override;
private:
    void RenderGUI();
    void RenderSimulationInfo();

    void RenderCubeParameters();
    void RenderCubeParametersDimension();
    void RenderCubeParametersDensity();
    void RenderCubeParametersAngularVelocity();

    void RenderTrajectoryParemeters();
    void RenderGravityParemeters();

    std::shared_ptr<RigidBodySimulation> simulation_;
    std::shared_ptr<RigidBodySimulationCreateParams> simulation_create_params_;

    std::shared_ptr<ifx::SceneWindowGUI> scene_window_gui_;
};


#endif //PROJECT_RIGID_BODY_GUI_H
