#ifndef PROJECT_RIGID_BODY_GUI_H
#define PROJECT_RIGID_BODY_GUI_H

#include <gui/gui.h>
#include <rigid_body_rotation/cube.h>

#include <memory>

namespace ifx{
class Renderer;
class EngineGUI;
}

struct Cube;
class RigidBodySimulation;

class RigidBodyGUI : public ifx::GUI {
public:

    RigidBodyGUI(GLFWwindow* window,
                 std::shared_ptr<RigidBodySimulation> simulation,
                 std::shared_ptr<ifx::Renderer> renderer);
    ~RigidBodyGUI();

    virtual void Render() override;
private:
    void RenderGUI();
    void RenderSimulationInfo();

    void RenderCubeParameters();
    void RenderCubeParametersDimension();
    void RenderCubeParametersDensity();
    void RenderCubeParametersAngularVelocity();
    void RenderCubeParametersDiagonalRotation();

    void RenderTrajectoryParemeters();
    void RenderGravityParemeters();

    std::shared_ptr<RigidBodySimulation> simulation_;
    Cube cube_;

    std::shared_ptr<ifx::EngineGUI> engine_gui_;
};


#endif //PROJECT_RIGID_BODY_GUI_H
