// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW2.h"

// Include any custom headers you created in your workspace
#include "MyBugAlgorithm.h"

using namespace amp;

#include<iostream>
#include<vector>
#include <string>
#include <cmath>

void print(std::vector <int> const &a) {
   std::cout << "The vector elements are : ";

   for(int i=0; i < a.size(); i++)
   std::cout << a.at(i) << ' ';
}

int main(int argc, char** argv) {

    /*    Randomly generate the problem     */ 

    // Use WO1 from Exercise 2
    Problem2D problem = HW2::getWorkspace1();

    // Use WO2 from Exercise 2
    /*
    Problem2D problem = HW2::getWorkspace2();
    */

    // Make a random environment spec, edit properties about it such as the number of obstacles
    /*
    Random2DEnvironmentSpecification spec;
    spec.max_obstacle_region_radius = 5.0;
    spec.n_obstacles = 2;
    spec.path_clearance = 0.01;
    spec.d_sep = 0.01;

    //Randomly generate the environment;
    Problem2D problem = EnvironmentTools::generateRandom(spec); // Random environment
    */

    // Declare your algorithm object 
    MyBugAlgorithm algo;
    
    {

        // Bug 1 or Bug 2
        int algorithm = 1;

        // Call your algorithm on the problem
        // amp::Path2D path = algo.plan(problem);
        amp::Path2D path = algo.plan(problem);


        // std::vector<Obstacle2D> obs = problem.obstacles;

        // for (const Obstacle2D& obstacle : obs) {
        //     const std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();
        //     for (const Eigen::Vector2d& vertex : vertices) {
        //         std::cout << '<' << vertex[0] << ',' << vertex[1] << '>';
        //     }
        //     std::cout << '\n';
        // }        

        // Check your path to make sure that it does not collide with the environment 
        bool success = HW2::check(path, problem);

        LOG("Found valid solution to workspace 1: " << (success ? "Yes!" : "No :("));

        // Visualize the path and environment
        Visualizer::makeFigure(problem, path);
    }

    // Let's get crazy and generate a random environment and test your algorithm
    {
        amp::Path2D path; // Make empty path, problem, and collision points, as they will be created by generateAndCheck()
        amp::Problem2D random_prob; 
        std::vector<Eigen::Vector2d> collision_points;
        bool random_trial_success = HW2::generateAndCheck(algo, path, random_prob, collision_points);
        LOG("Found valid solution in random environment: " << (random_trial_success ? "Yes!" : "No :("));

        LOG("path length: " << path.length());

        // Visualize the path environment, and any collision points with obstacles
        Visualizer::makeFigure(random_prob, path, collision_points);
    }

    Visualizer::showFigures();

    //HW2::grade(algo, "asra8222@colorado.edu", argc, argv);

    return 0;
}