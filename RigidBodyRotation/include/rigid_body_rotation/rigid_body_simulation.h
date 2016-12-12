#ifndef PROJECT_RIGID_BODY_SIMULATION_H
#define PROJECT_RIGID_BODY_SIMULATION_H

#include <math/math_ifx.h>
#include <vr/simulation.h>
#include <object/render_object.h>

#include <memory>


struct SceneFactoryObjects;

struct Cube {
    glm::vec3 dimensions;
    float density;
    float angular_velocity_initial;
    float diagonal_rotation_initial;
    float gravity_force;
    bool gravity_on;

    std::shared_ptr<ifx::RenderObject> render_object;
    std::shared_ptr<ifx::RenderObject> diagonal_render_object;
    std::shared_ptr<ifx::RenderObject> force_render_object;

    glm::vec3 GetRotationAngles(){
        return glm::vec3(0,
                         glm::degrees(atan(1L)),
                         glm::degrees(asin(sqrt(2L)/sqrt(3L))));
    };

    // From origin (x,y,z) to starting rotated frame (X,Y,Z).
    // Change of basis matrix, where diagonal is aligned with y axis
    glm::mat3 GetRotationMatrix(){
        return Rz(glm::degrees(asin(sqrt(2L)/sqrt(3L)))) *
                Ry(glm::degrees(atan(1L))) *
                Rx(0L);
    };

    glm::quat GetRotationQuat(){
        return glm::normalize(glm::quat(glm::radians(GetRotationAngles())));
    }

    glm::vec3 GetDiagonalVector(){
        return glm::vec3(0L, 1L, 0L);
    }

    glm::mat3 Rx(double alpha){
        glm::mat3 mat;
        mat[0].x = 1;
        mat[0].y = 0;
        mat[0].z = 0;

        mat[1].x = 0;
        mat[1].y = cos(alpha);
        mat[1].z = sin(alpha);

        mat[2].x = 0;
        mat[2].y = -sin(alpha);
        mat[2].z = cos(alpha);

        return mat;
    }
    glm::mat3 Ry(double alpha){
        glm::mat3 mat;
        mat[0].x = cos(alpha);
        mat[0].y = 0;
        mat[0].z = -sin(alpha);

        mat[1].x = 0;
        mat[1].y = 1;
        mat[1].z = 0;

        mat[2].x = sin(alpha);
        mat[2].y = 0;
        mat[2].z = cos(alpha);

        return mat;
    }
    glm::mat3 Rz(double alpha){
        glm::mat3 mat;
        mat[0].x = cos(alpha);
        mat[0].y = sin(alpha);
        mat[0].z = 0;

        mat[1].x = -sin(alpha);
        mat[1].y = cos(alpha);
        mat[1].z = 0;

        mat[2].x = 0;
        mat[2].y = 0;
        mat[2].z = 1;

        return mat;
    }
};

struct InertiaData{
    // The initial change of basis
    glm::mat3 change_of_basis_matrix;

    glm::mat3 inertia_tensor_body;
    glm::vec3 center_body;
    glm::vec3 diagonal_body;
    glm::vec3 force;

    // along transformed diagonal
    glm::vec3 angular_velocity_current;
    glm::quat quat_rotation_current;
};

struct RigidBodySimulationCreateParams{
    glm::vec3 dimensions;
    float density;
    float angular_velocity;

    float diagonal_rotation;

    float gravity_force;
    bool gravity_on;
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

    Cube* cube(){return &cube_;}
    bool gravity_on(){return grivity_on_;}
    void gravity_on(bool val){grivity_on_ = val;}
    double total_time(){return time_data_.total_time;}

    void Reset(std::shared_ptr<RigidBodySimulationCreateParams> create_params);

    virtual void Update() override;
private:
    void Update(double elapsed);

    void SetSceneObjects(SceneFactoryObjects& scene_factory_objects);
    std::shared_ptr<RigidBodySimulationCreateParams> GetDefaultParameters();

    void ResetTimeData();
    void ResetCubeData(
            std::shared_ptr<RigidBodySimulationCreateParams> create_params);
    void ResetInertiaTensorData();
    void ResetViewObjects();

    Cube cube_;
    InertiaData inertia_data_;
    TimeData time_data_;

    bool grivity_on_;
};


#endif //PROJECT_RIGID_BODY_SIMULATION_H
