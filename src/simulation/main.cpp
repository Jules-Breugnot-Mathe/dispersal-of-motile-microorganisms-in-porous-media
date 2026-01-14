#include <iostream>
#include <vector>
#include <memory>
#include <random>

//bibliothèques graphiques
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>


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
#include "InfiniteEnvironmentRenderer.hpp"


using namespace std;



// callbacks
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}

int main()
{
    
    Environment E("uniform_Disks");
    Environment F("uniform_Squares");

    Mobile M(Point(0,0), 0.1, 1, 1, 0, 0.5); // (Point, v, Dr, tau, Theta, mu)

    M.InfiniteSimulationRenderer(E, 0.01, Point(0.0, 0.0), 0, "isotropic");
    M.PeriodicSimulationRenderer(E, 0.01, Point(0.0, 0.0), 0, "isotropic");
    M.PeriodicSimulationRenderer(F, 0.01, Point(0.0, 0.0), 0, "isotropic");
    

    return 0;
    
}


