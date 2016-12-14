#ifndef PROJECT_RIGID_BODY_SIMULATION_H
#define PROJECT_RIGID_BODY_SIMULATION_H

#include <math/math_ifx.h>
#include <vr/simulation.h>
#include <object/render_object.h>
#include <rigid_body_rotation/cube.h>

#include <memory>

namespace ifx{
class Scene;
}

struct SceneFactoryObjects;

struct Views {
    std::shared_ptr<ifx::RenderObject> cube;
    std::shared_ptr<ifx::RenderObject> diagonal;
    std::shared_ptr<ifx::RenderObject> force;

    std::shared_ptr<ifx::RenderObject> trajectory;
};

struct InertiaData{
    glm::highp_dmat3 inertia_tensor_body;
    glm::highp_dmat3 inertia_tensor_inv_body;

    glm::highp_dvec3 center_body;
    glm::highp_dvec3 diagonal_body;
    glm::highp_dvec3 force;

    // along transformed diagonal
    glm::highp_dvec3 angular_velocity_current;
    glm::highp_dquat quat_rotation_current;

    glm::highp_dvec3 GetCubeRotation(){
        auto Q = quat_rotation_current * GetRotationQuatInitial();
        auto current = glm::degrees(glm::eulerAngles((Q)));
        return current;
/*
        auto Q = quat_rotation_current;
        auto current = glm::degrees(glm::eulerAngles((Q)));
        return current;*/
    }

    glm::highp_dvec3 GetDiagonalRotation(){
        /*
        auto Q = quat_rotation_current * glm::inverse(GetRotationQuatInitial());
        auto current = glm::degrees(glm::eulerAngles((Q)));*/

        auto current = glm::degrees(glm::eulerAngles(quat_rotation_current));
        return current;
    }

    glm::highp_dvec3 GetForceRotation(){
        auto initial = glm::highp_dvec3(180,0,0);
        auto current
                = -glm::degrees(
                        glm::eulerAngles((glm::inverse(quat_rotation_current))));
        return initial + current;
    }
};

struct TimeData{
    double last_time;
    double current_time;

    double total_time;

    double time_since_last_update;
    const double time_delta = 1.0f / 100.0f;
};

class RigidBodySimulation : public ifx::Simulation {
public:

    RigidBodySimulation(SceneFactoryObjects& scene_factory_objects,
                        std::shared_ptr<ifx::Scene> scene);
    ~RigidBodySimulation();

    Cube& cube(){return cube_;}
    double total_time(){return time_data_.total_time;}
    bool* show_trajectory(){return &(cube_.render_trajectory);}

    void Reset(Cube& cube);
    virtual void Update() override;

private:
    void Update(double elapsed);
    void UpdateViewObjects();

    /**
     * Reset the simulation
     */
    void ResetTimeData();
    void ResetCubeData();
    void ResetInertiaTensorData();

    void SetSceneObjects(SceneFactoryObjects& scene_factory_objects);
    Cube GetDefaultCube();

    void UpdateTrajectory();
    void ResetTrajectoryView();

    Cube cube_;
    InertiaData inertia_data_;
    TimeData time_data_;
    Views views_;

    std::vector<glm::vec3> trajectory_;
    std::shared_ptr<ifx::Scene> scene_;
};


#endif //PROJECT_RIGID_BODY_SIMULATION_H
