// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework header
#include "hw/HW5.h"
#include "hw/HW2.h"

#include "gradientDescent.h"

int main(int argc, char** argv) {
    // // Use WO1 from Exercise 2
    // amp::Problem2D problem = amp::HW2::getWorkspace1();

    // // Use WO2 from Exercise 2
    // amp::Problem2D problem = amp::HW2::getWorkspace2();

    // // Use WO1 from Exercise 5
    amp::Problem2D problem = amp::HW5::getWorkspace1();
    

    double dstar = 3; 
    double Qstar = 0.5;
    double zeta = 0.1;
    double eta = 0.05;
    amp::MyGradientDescent gd(dstar, Qstar, zeta, eta);
    amp::Path2D path = gd.plan(problem);
    std::cout << "Path Length: " << path.length() << std::endl;
    amp::Visualizer::makeFigure(problem, path);
    amp::Visualizer::showFigures();
    // Grade method
    // amp::HW5::grade(const std::string& email, int argc, char** argv, CTOR_ARGS_T&&... constructor_arguments);
    amp::HW5::grade(gd, "asra8222@colorado.edu", argc, argv);
    return 0;
}


