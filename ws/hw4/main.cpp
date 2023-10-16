// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW4.h"

// Include the header of the shared class
#include "HelpfulClass.h"

// Include classes for running
#include <iostream>
#include <cmath>

#include "cSpace.h"
#include "DerivedLinkManipulator.h"

using namespace amp;

void problem1a(){
    Obstacle2D obstacle =  HW4::getEx1TriangleObstacle();
    std::vector<Eigen::Vector2d> robot = {Eigen::Vector2d(0,0), Eigen::Vector2d(-1,-2), Eigen::Vector2d(0,-2)};
    
    tools helper;
    Obstacle2D cspaceObstacle = helper.computeMinkowskiSum(robot, obstacle.verticesCCW());

    // Print the C-space obstacle vertices
    for (const auto& vertex : cspaceObstacle.verticesCCW()) {
        std::cout << "(" << vertex[0] << ", " << vertex[1] << ")" << std::endl;
    }
    Visualizer::makeFigure({cspaceObstacle.verticesCCW()}, true); // filled set to true to visualize it as a filled polygon
    Visualizer::showFigures();
}

void problem1b(){
    Obstacle2D obstacle = HW4::getEx1TriangleObstacle();
    std::vector<Eigen::Vector2d> robot = {Eigen::Vector2d(0,0), Eigen::Vector2d(-1,-2), Eigen::Vector2d(0,-2)};
    tools helper;

    std::vector<Polygon> polygons; // To hold all the C-space obstacles for different rotations
    std::vector<double> heights; // To hold the theta values (heights in 3D visualization)

    for (int i = 0; i < 12; ++i) {
        double theta = (2 * M_PI / 12) * i;

        // Rotate the robot vertices
        std::vector<Eigen::Vector2d> rotatedRobot;
        for (const auto& vertex : robot) {
            double x = vertex[0] * cos(theta) - vertex[1] * sin(theta);
            double y = vertex[0] * sin(theta) + vertex[1] * cos(theta);
            rotatedRobot.push_back(Eigen::Vector2d(x, y));
        }

        Obstacle2D cspaceObstacle = helper.computeMinkowskiSum(rotatedRobot, obstacle.verticesCCW());
        polygons.push_back(cspaceObstacle.verticesCCW());
        heights.push_back(theta);
    }

    // Use the Visualizer to plot
    Visualizer::makeFigure(polygons, heights);
    Visualizer::showFigures();
}

void problem2a(){
    std::vector<double> linkLengths = {0.5, 1, 0.5};

    ManipulatorState state = {M_PI/6, M_PI/3, 7*M_PI/4};

    LinkManipulator manipulator(linkLengths);

    Visualizer::makeFigure(manipulator, state);

    Visualizer::showFigures();
    
}

void problem2b(){
    std::vector<double> linkLengths = {1, 0.5, 1};

    Eigen::Vector2d end_effector_location(2.0,0.0);

    LinkManipulator manipulator(linkLengths);

    ManipulatorState state = manipulator.getConfigurationFromIK(end_effector_location);

    Visualizer::makeFigure(manipulator, state);

    Visualizer::showFigures();
    
}


void problem3a(){

    Environment2D ws = HW4::getEx3Workspace1();
    std::vector<amp::Obstacle2D> obstacles = ws.obstacles;
    double x0_min = ws.x_min;
    double x0_max = ws.x_max;
    double x1_min = ws.y_min;
    double x1_max = ws.y_max;

    std::size_t x0_cells = 100;
    std::size_t x1_cells = 100;

    std::vector<double> linkLengths = {1, 1, 1};
    LinkManipulator manipulator(linkLengths);

    CSpace2DLinkManipulator cSpace(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max, manipulator, obstacles);

    Visualizer::makeFigure(cSpace);

    Visualizer::showFigures();
    
}

void problem3b(){

    Environment2D ws = HW4::getEx3Workspace2();
    std::vector<amp::Obstacle2D> obstacles = ws.obstacles;
    double x0_min = ws.x_min;
    double x0_max = ws.x_max;
    double x1_min = ws.y_min;
    double x1_max = ws.y_max;

    std::size_t x0_cells = 100;
    std::size_t x1_cells = 100;

    std::vector<double> linkLengths = {1, 1, 1};
    LinkManipulator manipulator(linkLengths);

    CSpace2DLinkManipulator cSpace(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max, manipulator, obstacles);

    Visualizer::makeFigure(cSpace);

    Visualizer::showFigures();
    
}

void problem3c(){

    Environment2D ws = HW4::getEx3Workspace3();
    std::vector<amp::Obstacle2D> obstacles = ws.obstacles;
    double x0_min = ws.x_min;
    double x0_max = ws.x_max;
    double x1_min = ws.y_min;
    double x1_max = ws.y_max;

    std::size_t x0_cells = 100;
    std::size_t x1_cells = 100;

    std::vector<double> linkLengths = {1, 1, 1};
    LinkManipulator manipulator(linkLengths);

    CSpace2DLinkManipulator cSpace(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max, manipulator, obstacles);

    Visualizer::makeFigure(cSpace);

    Visualizer::showFigures();
    
}


int main(int argc, char** argv) {
    /* Include this line to have different randomized environments every time you run your code (NOTE: this has no affect on grade()) */
    // amp::RNG::seed(amp::RNG::randiUnbounded());

    // problem1a();
    // problem1b();
    // problem2a();
    // problem2b();
    problem3a();
    // problem3b();
    // problem3c();

    // CSpace2DLinkManipulator constructor{};
    // LinkManipulator MyLinkManipulator;

    // Grade method
    // amp::HW4::grade<MyLinkManipulator>(constructor, "asra8222@colorado.edu", argc, argv);
    return 0;
}