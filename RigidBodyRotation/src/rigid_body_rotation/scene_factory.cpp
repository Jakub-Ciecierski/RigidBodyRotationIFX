#include "rigid_body_rotation/scene_factory.h"

#include <object/render_object.h>
#include <factory/render_object_factory.h>
#include <factory/texture_factory.h>
#include <factory/program_factory.h>
#include <model_loader/model_loader.h>

SceneFactory::SceneFactory(){

}

SceneFactory::~SceneFactory(){

}

SceneFactoryObjects SceneFactory::CreateSceneObjects(){
    SceneFactoryObjects scene_factory_objects;

    scene_factory_objects.cube = CreateCube();
    scene_factory_objects.diagonal = CreateDiagonalVector();
    scene_factory_objects.axis = CreateAxis();
    scene_factory_objects.gravity_vector = CreateGravityVector();
    scene_factory_objects.gravity_plane = CreateGravityPlane();

    return scene_factory_objects;
}

std::shared_ptr<ifx::RenderObject> SceneFactory::CreateCube(){
    std::shared_ptr<Program> program = ifx::ProgramFactory().LoadMainProgram();
    auto render_object
            = std::shared_ptr<ifx::RenderObject>(new ifx::RenderObject(
                    ObjectID(0, "Cube"),
                    CreateCubeModel()));

    render_object->addProgram(program);
    render_object->rotateTo(glm::vec3(0,
                                      glm::degrees(atan(1)),
                                      glm::degrees(asin(sqrt(2L)/sqrt(3L)))));

    return render_object;
}

std::shared_ptr<ifx::RenderObject> SceneFactory::CreateDiagonalVector(){
    std::shared_ptr<Program> program = ifx::ProgramFactory().LoadMainProgram();

    std::string path
            = ifx::Resources::GetInstance().GetResourcePath(
                    "vectors/y/vector.obj", ifx::ResourceType::MODEL);
    auto model = ifx::ModelLoader(path).loadModel();
    auto render_object
            = std::shared_ptr<ifx::RenderObject>(new ifx::RenderObject(
                    ObjectID(0, "Diagonal"), model));
    render_object->addProgram(program);
    render_object->scale(glm::vec3(1, 1*sqrt(3),1));

    auto q = glm::quat(glm::vec3(0, (atan(1L)), (asin(sqrt(2L)/sqrt(3L)))));
    q = glm::normalize(q);
    render_object->rotateTo(glm::degrees(glm::eulerAngles(glm::inverse(q))));

    return render_object;
}

std::shared_ptr<ifx::RenderObject> SceneFactory::CreateAxis(){
    std::shared_ptr<Program> program = ifx::ProgramFactory().LoadMainProgram();

    auto model = CreateAxisModel();
    auto render_object
            = std::shared_ptr<ifx::RenderObject>(new ifx::RenderObject(
                    ObjectID(0, "Axis"), model));

    render_object->addProgram(program);
    render_object->scale(4.0f);

    render_object->SetBeforeRender(
            [](const Program *program){
                glLineWidth(5);
            }
    );
    render_object->SetAfterRender(
            [](const Program *program){
                glLineWidth(1);
            }
    );

    return render_object;
}

std::shared_ptr<ifx::RenderObject> SceneFactory::CreateGravityVector(){
    std::shared_ptr<Program> program = ifx::ProgramFactory().LoadMainProgram();

    std::string path
            = ifx::Resources::GetInstance().GetResourcePath(
                    "vectors/x/vector.obj", ifx::ResourceType::MODEL);
    auto model = ifx::ModelLoader(path).loadModel();
    auto render_object
            = std::shared_ptr<ifx::RenderObject>(new ifx::RenderObject(
                    ObjectID(0, "Force"), model));
    render_object->addProgram(program);
    render_object->rotateTo(glm::vec3(180,0,0));

    return render_object;
}

std::shared_ptr<ifx::RenderObject> SceneFactory::CreateGravityPlane(){
    return std::shared_ptr<ifx::RenderObject>();
}

std::unique_ptr<ifx::Mesh> SceneFactory::CreateCubeMesh() {
    // Position, Normal, TexCoord
    std::vector <Vertex> vertices = {
            // Front
            Vertex{glm::vec3(1.0f, 1.0f, 0.0f),
                   glm::vec3(0.0f, 0.0f, -1.0f), glm::vec2(1.0f, 1.0f)},
            Vertex{glm::vec3(1.0f, 0.0f, 0.0f),
                   glm::vec3(0.0f, 0.0f, -1.0f), glm::vec2(1.0f, 0.0f)},
            Vertex{glm::vec3(0.0f, 0.0f, 0.0f),
                   glm::vec3(0.0f, 0.0f, -1.0f), glm::vec2(0.0f, 0.0f)},
            Vertex{glm::vec3(0.0f, 1.0f, 0.0f),
                   glm::vec3(0.0f, 0.0f, -1.0f), glm::vec2(0.0f, 1.0f)},

            // Back
            Vertex{glm::vec3(1.0f, 1.0f, 1.0f),
                   glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(1.0f, 1.0f)},
            Vertex{glm::vec3(1.0f, 0.0f, 1.0f),
                   glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(1.0f, 0.0f)},
            Vertex{glm::vec3(0.0f, 0.0f, 1.0f),
                   glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(0.0f, 0.0f)},
            Vertex{glm::vec3(0.0f, 1.0f, 1.0f),
                   glm::vec3(0.0f, 0.0f, 1.0f), glm::vec2(0.0f, 1.0f)},

            // Left
            Vertex{glm::vec3(0.0f, 1.0f, 1.0f),
                   glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec2(1.0f, 1.0f)},
            Vertex{glm::vec3(0.0f, 0.0f, 1.0f),
                   glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec2(1.0f, 0.0f)},
            Vertex{glm::vec3(0.0f, 0.0f, 0.0f),
                   glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec2(0.0f, 0.0f)},
            Vertex{glm::vec3(0.0f, 1.0f, 0.0f),
                   glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec2(0.0f, 1.0f)},

            // Right
            Vertex{glm::vec3(1.0f, 1.0f, 1.0f),
                   glm::vec3(1.0f, 0.0f, 0.0f), glm::vec2(1.0f, 1.0f)},
            Vertex{glm::vec3(1.0f, 0.0f, 1.0f),
                   glm::vec3(1.0f, 0.0f, 0.0f), glm::vec2(1.0f, 0.0f)},
            Vertex{glm::vec3(1.0f, 0.0f, 0.0f),
                   glm::vec3(1.0f, 0.0f, 0.0f), glm::vec2(0.0f, 0.0f)},
            Vertex{glm::vec3(1.0f, 1.0f, 0.0f),
                   glm::vec3(1.0f, 0.0f, 0.0f), glm::vec2(0.0f, 1.0f)},

            // Bottom
            Vertex{glm::vec3(1.0f, 0.0f, 1.0f),
                   glm::vec3(0.0f, -1.0f, 0.0f), glm::vec2(1.0f, 1.0f)},
            Vertex{glm::vec3(1.0f, 0.0f, 0.0f),
                   glm::vec3(0.0f, -1.0f, 0.0f), glm::vec2(1.0f, 0.0f)},
            Vertex{glm::vec3(0.0f, 0.0f, 0.0f),
                   glm::vec3(0.0f, -1.0f, 0.0f), glm::vec2(0.0f, 0.0f)},
            Vertex{glm::vec3(0.0f, 0.0f, 1.0f),
                   glm::vec3(0.0f, -1.0f, 0.0f), glm::vec2(0.0f, 1.0f)},

            // Top
            Vertex{glm::vec3(1.0f, 1.0f, 1.0f),
                   glm::vec3(0.0f, 1.0f, 0.0f), glm::vec2(1.0f, 1.0f)},
            Vertex{glm::vec3(1.0f, 1.0f, 0.0f),
                   glm::vec3(0.0f, 1.0f, 0.0f), glm::vec2(1.0f, 0.0f)},
            Vertex{glm::vec3(0.0f, 1.0f, 0.0f),
                   glm::vec3(0.0f, 1.0f, 0.0f), glm::vec2(0.0f, 0.0f)},
            Vertex{glm::vec3(0.0f, 1.0f, 1.0f),
                   glm::vec3(0.0f, 1.0f, 0.0f), glm::vec2(0.0f, 1.0f)},
    };

    std::vector <GLuint> indices = {
            3, 1, 0, 3, 2, 1,            // front
            4, 5, 7, 5, 6, 7,            // back

            8, 9, 11, 9, 10, 11,           // left
            15, 13, 12, 15, 14, 13,        // right

            16, 17, 19, 17, 18, 19,        // bottom
            23, 21, 20, 23, 22, 21,        // top
    };

    std::unique_ptr<ifx::Mesh> mesh(new ifx::Mesh(vertices, indices));
    auto material = std::make_shared<ifx::Material>();
    material->alpha = 0.3f;
    material->AddTexture(ifx::TextureFactory().LoadContainerDiffuse());
    material->AddTexture(ifx::TextureFactory().LoadContainerSpecular());

    mesh->material(material);

    return mesh;
}

std::shared_ptr<ifx::Model> SceneFactory::CreateCubeModel() {
    std::vector<std::unique_ptr<ifx::Mesh>> meshes;
    meshes.push_back(std::move(CreateCubeMesh()));

    return ifx::Model::MakeModel(ifx::NO_FILEPATH, std::move(meshes));
}

std::shared_ptr<ifx::Model> SceneFactory::CreateAxisModel(){
    std::vector<Vertex> vertices1{
            Vertex{glm::vec3(0.0f, 0.0f, 0.0f),
                   glm::vec3(0.0f, 0.0f, -1.0f),
                   glm::vec2(0.0f, 0.0f)},
            Vertex{glm::vec3(1.0f, 0.0f, 0.0f),
                   glm::vec3(0.0f, 0.0f, -1.0f),
                   glm::vec2(1.0f, 1.0f)}
    };
    std::vector <GLuint> indices1 = { 0,1 };

    std::vector<Vertex> vertices2{
            Vertex{glm::vec3(0.0f, 0.0f, 0.0f),
                   glm::vec3(0.0f, 0.0f, -1.0f),
                   glm::vec2(0.0f, 0.0f)},
            Vertex{glm::vec3(0.0f, 1.0f, 0.0f),
                   glm::vec3(0.0f, 0.0f, -1.0f),
                   glm::vec2(1.0f, 1.0f)},

    };
    std::vector <GLuint> indices2 = { 0,1 };

    std::vector<Vertex> vertices3{
            Vertex{glm::vec3(0.0f, 0.0f, 0.0f),
                   glm::vec3(0.0f, 0.0f, -1.0f),
                   glm::vec2(0.0f, 0.0f)},
            Vertex{glm::vec3(0.0f, 0.0f, 1.0f),
                   glm::vec3(0.0f, 0.0f, -1.0f),
                   glm::vec2(1.0f, 1.0f)},


    };
    std::vector <GLuint> indices3 = { 0,1 };

    std::unique_ptr<ifx::Mesh> mesh1(new ifx::Mesh(vertices1, indices1));
    std::unique_ptr<ifx::Mesh> mesh2(new ifx::Mesh(vertices2, indices2));
    std::unique_ptr<ifx::Mesh> mesh3(new ifx::Mesh(vertices3, indices3));

    auto material1 = std::make_shared<ifx::Material>();
    material1->AddTexture(ifx::TextureFactory().CreateSolidColorTexture(
            glm::vec3(255,0,0), ifx::TextureTypes::DIFFUSE));
    material1->AddTexture(ifx::TextureFactory().CreateSolidColorTexture(
            glm::vec3(255,0,0), ifx::TextureTypes::SPECULAR));

    mesh1->material(material1);
    mesh1->primitive_draw_mode(ifx::PrimitiveDrawMode::LINES);

    auto material2 = std::make_shared<ifx::Material>();
    material2->AddTexture(ifx::TextureFactory().CreateSolidColorTexture(
            glm::vec3(0,255,0), ifx::TextureTypes::DIFFUSE));
    material2->AddTexture(ifx::TextureFactory().CreateSolidColorTexture(
            glm::vec3(0,255,0), ifx::TextureTypes::SPECULAR));

    mesh2->material(material2);
    mesh2->primitive_draw_mode(ifx::PrimitiveDrawMode::LINES);

    auto material3 = std::make_shared<ifx::Material>();
    material3->AddTexture(ifx::TextureFactory().CreateSolidColorTexture(
            glm::vec3(0,0,255), ifx::TextureTypes::DIFFUSE));
    material3->AddTexture(ifx::TextureFactory().CreateSolidColorTexture(
            glm::vec3(0,0,255), ifx::TextureTypes::SPECULAR));

    mesh3->material(material3);
    mesh3->primitive_draw_mode(ifx::PrimitiveDrawMode::LINES);

    std::vector<std::unique_ptr<ifx::Mesh>> meshes;
    meshes.push_back(std::move(mesh1));
    meshes.push_back(std::move(mesh2));
    meshes.push_back(std::move(mesh3));

    return ifx::Model::MakeModel(ifx::NO_FILEPATH, std::move(meshes));
}