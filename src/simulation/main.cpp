#include <iostream>
#include <vector>
#include <memory>
#include <random>

//bibliothèques graphiques
#include <glad/glad.h>
#include <GLFW/glfw3.h>

//classes numériques
#include "Point.hpp"
#include "Solid.hpp"
#include "Disk.hpp"
#include "Mobile.hpp"
#include "Environment.hpp"
#include "Rectangle.hpp"
#include "Domain.hpp"
#include "tests.hpp"

//classes graphiques
#include "Shader.hpp"
#include "Renderer.hpp"
#include "DiskRenderer.hpp"
#include "RectangleRenderer.hpp"
#include "MobileRenderer.hpp"
#include "RectangleRenderer.hpp"
#include "TrajectoryRenderer.hpp"
#include "EnvironmentRenderer.hpp"


using namespace std;



// callbacks
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}

int main()
{
    
    Environment E("uniform_Disks");
    Mobile M(Point(0,0), 0.1, 1, 1, 0, 0.5); // (Point, v, Dr, tau, Theta, mu)
    M.PeriodicSimulationRenderer(E, 0.01, Point(0.0, 0.0), 0, "isotropic");
    

    /*
    //=============================INITIALISATION + CONFIGURATION GL================================//    
    // ---------------- GLFW INIT ----------------
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return -1;
    }
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(800, 600, "Static Environment Simulation", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create window\n";
        glfwTerminate();
        return-1;
    }
    glfwMakeContextCurrent(window);
    //glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    // ---------------- GLAD ----------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD\n";
        return -1;
    }
    // ---------------- SHADER ----------------
    Shader shader(
        "shaders/basic.vert",
        "shaders/basic.frag"
    );
    //------------ENVIRONMENT RENDERER-------------
    Environment E("uniform_Disks");
    EnvironmentRenderer envrenderer(E);

    //------------RENDERING LOOP----------------
    while(glfwWindowShouldClose(window) == 0) {

        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window, true);

        glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        envrenderer.draw(shader);


        glfwSwapBuffers(window);
        glfwPollEvents();

    }
    
    glfwTerminate();
    */

    return 0;
}


