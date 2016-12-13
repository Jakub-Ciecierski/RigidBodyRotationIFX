#ifndef PROJECT_RIGID_BODY_SIMULATION_H
#define PROJECT_RIGID_BODY_SIMULATION_H

#include <math/math_ifx.h>
#include <vr/simulation.h>
#include <object/render_object.h>
#include <rigid_body_rotation/cube.h>

#include <memory>

struct SceneFactoryObjects;

struct Views {
    std::shared_ptr<ifx::RenderObject> cube;
    std::shared_ptr<ifx::RenderObject> diagonal;
    std::shared_ptr<ifx::RenderObject> force;
};

struct InertiaData{
    glm::mat3 inertia_tensor_body;
    glm::mat3 inertia_tensor_inv_body;

    glm::vec3 center_body;
    glm::vec3 diagonal_body;
    glm::vec3 force;

    // along transformed diagonal
    glm::vec3 angular_velocity_current;
    glm::quat quat_rotation_current;

    glm::quat qt;

    glm::vec3 GetCubeRotation(){
        auto Q = quat_rotation_current * GetRotationQuatInitial();
        auto current = glm::degrees(glm::eulerAngles((Q)));
        return current;
/*
        auto Q = quat_rotation_current;
        auto current = glm::degrees(glm::eulerAngles((Q)));
        return current;*/
    }

    glm::vec3 GetDiagonalRotation(){
        /*
        auto Q = quat_rotation_current * glm::inverse(GetRotationQuatInitial());
        auto current = glm::degrees(glm::eulerAngles((Q)));*/

        auto current = glm::degrees(glm::eulerAngles(quat_rotation_current));
        return current;
    }

    glm::vec3 GetForceRotation(){
        auto initial = glm::vec3(180,0,0);
        auto current
                = -glm::degrees(
                        glm::eulerAngles((glm::inverse(quat_rotation_current))));
        return initial + current;
    }
};

struct TimeData{
    float last_time;
    float current_time;

    float total_time;

    float time_since_last_update;
    const float time_delta = 1.0f / 60.0f;
};

class RigidBodySimulation : public ifx::Simulation {
public:

    RigidBodySimulation(SceneFactoryObjects& scene_factory_objects);
    ~RigidBodySimulation();

    Cube& cube(){return cube_;}
    double total_time(){return time_data_.total_time;}

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

    Cube cube_;
    InertiaData inertia_data_;
    TimeData time_data_;
    Views views_;
};


#endif //PROJECT_RIGID_BODY_SIMULATION_H
