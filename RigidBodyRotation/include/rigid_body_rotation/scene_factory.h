#ifndef RIGID_BODY_SCENE_FACTORY_H
#define RIGID_BODY_SCENE_FACTORY_H

#include <memory>

namespace ifx{
class Mesh;
class Model;
class RenderObject;
}

struct SceneFactoryObjects {
    std::shared_ptr<ifx::RenderObject> cube;
    std::shared_ptr<ifx::RenderObject> axis;
    std::shared_ptr<ifx::RenderObject> gravity_vector;
    std::shared_ptr<ifx::RenderObject> gravity_plane;
};

class SceneFactory {
public:

    SceneFactory();
    ~SceneFactory();

    SceneFactoryObjects CreateSceneObjects();

private:
    std::shared_ptr<ifx::RenderObject> CreateCube();
    std::shared_ptr<ifx::RenderObject> CreateAxis();
    std::shared_ptr<ifx::RenderObject> CreateGravityVector();
    std::shared_ptr<ifx::RenderObject> CreateGravityPlane();

    std::unique_ptr<ifx::Mesh> CreateCubeMesh();
    std::shared_ptr<ifx::Model> CreateCubeModel();

    std::shared_ptr<ifx::Model> CreateAxisModel();
};


#endif //RIGID_BODY_SCENE_FACTORY_H
