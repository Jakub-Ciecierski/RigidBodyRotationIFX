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
    glm::vec3 angular_velocity;
    std::shared_ptr<ifx::RenderObject> render_object;

    glm::mat3 GetRotationMatrix(){
        return Rz(render_object->getRotation().z) *
                Ry(render_object->getRotation().y) *
                Rx(render_object->getRotation().x);
    };

    glm::mat3 Rx(float alpha){
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
    glm::mat3 Ry(float alpha){
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
    glm::mat3 Rz(float alpha){
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

struct RigidBodySimulationCreateParams{
    glm::vec3 dimensions;
    float density;
    glm::vec3 angular_velocity;
};

class RigidBodySimulation : public ifx::Simulation {
public:

    RigidBodySimulation(SceneFactoryObjects& scene_factory_objects);
    ~RigidBodySimulation();

    Cube* cube(){return &cube_;}
    bool gravity_on(){return grivity_on_;}
    void gravity_on(bool val){grivity_on_ = val;}
    double total_time(){return total_time_s_;}

    void Reset(std::shared_ptr<RigidBodySimulationCreateParams> create_params);

    virtual void Update() override;
private:
    void SetDefaultParameters(SceneFactoryObjects& scene_factory_objects);

    bool SatisfiesTimeDelta();

    void SetSceneObjects(SceneFactoryObjects& scene_factory_objects);

    std::shared_ptr<RigidBodySimulationCreateParams> GetDefaultParameters();

    void PrintTensorInfo();

    struct Cube cube_;

    bool grivity_on_;

    // In seconds.
    double time_delta_;
    double time_delta_sqr_;
    double current_update_time_;
    double last_update_time_;
    double total_time_s_;
};


#endif //PROJECT_RIGID_BODY_SIMULATION_H
