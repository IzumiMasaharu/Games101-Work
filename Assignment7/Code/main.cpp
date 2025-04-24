#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Matrix.hpp"
#include "Sphere.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <chrono>

// In the main function of the program, we create the scene (create objects and
// lights) as well as set the options for the render (image width and height,
// maximum recursion depth, field-of-view, etc.). We then call the render
// function().
int main(int argc, char** argv)
{
    // Change the definition here to change resolution
    Scene scene(784, 784);

    Material* light = new Material(DIFFUSE, (8.0f * Vector3f(0.747f+0.058f, 0.747f+0.258f, 0.747f) + 15.6f * Vector3f(0.740f+0.287f,0.740f+0.160f,0.740f) + 18.4f *Vector3f(0.737f+0.642f,0.737f+0.159f,0.737f)),Vector3f(0.65f));
    Material* red = new Material(DIFFUSE, Vector3f(0.0f),Vector3f(0.63f, 0.065f, 0.05f));
    Material* green = new Material(DIFFUSE, Vector3f(0.0f),Vector3f(0.14f, 0.45f, 0.091f));
    Material* white = new Material(DIFFUSE, Vector3f(0.0f),Vector3f(0.725f, 0.71f, 0.68f));

    Material* yellow_rubber = new Material(
        MICROFACET, 
        Vector3f(0.0f),        
        1.5f,                  
        Vector3f(0.6f, 0.4f, 0.2f), 
        Vector3f(0.05f, 0.05f, 0.05f), 
        0.8f                    
    );

    MeshTriangle light_("../models/light.obj", light);
    MeshTriangle left("../models/left.obj", red);
    MeshTriangle right("../models/right.obj", green);
    MeshTriangle floor("../models/floor.obj", white);
    MeshTriangle top("../models/top.obj", white);
    MeshTriangle back("../models/back.obj", white);
    // MeshTriangle shortbox("../models/shortbox.obj", white);
    // MeshTriangle tallbox("../models/tallbox.obj", white);
    Matrix4f modelMatrix = Matrix4f::Translate(400, 0, 350) * Matrix4f::Scale(15.0f, 15.0f, 15.0f) * Matrix4f::RotateY(225);
    MeshTriangle nailong("../models/Nailong.obj", yellow_rubber, modelMatrix);
    modelMatrix = Matrix4f::Translate(175, 0, 350) * Matrix4f::Scale(300.0f, 300.0f, 300.0f) *Matrix4f::RotateY(150)*Matrix4f::RotateX(-90);
    MeshTriangle HanabiBomb("../models/HanabiBomb.obj", white, modelMatrix);
    
    scene.Add(&light_);
    scene.Add(&left);
    scene.Add(&right);
    scene.Add(&floor);
    scene.Add(&top);
    scene.Add(&back);
    // scene.Add(&shortbox);
    // scene.Add(&tallbox);
    scene.Add(&nailong);
    scene.Add(&HanabiBomb);

    scene.buildBVH();

    Renderer r;

    auto start = std::chrono::system_clock::now();
    r.Render(scene);
    auto stop = std::chrono::system_clock::now();

    std::cout << "Render complete: \n";
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";

    return 0;
}