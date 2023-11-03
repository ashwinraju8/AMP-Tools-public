#include "AMPCore.h"
#include "hw/HW7.h"
#include "hw/HW2.h"
#include "hw/HW5.h"
#include "MyPRM.h"
#include "MyRRT.h"

using namespace amp;

void runBenchmark(MyPRM2D& planner, const amp::Problem2D& problem, int runs, std::vector<std::pair<int, double>> nr_pairs) {
    std::map<std::string, std::vector<double>> path_lengths;
    std::map<std::string, std::vector<double>> computation_times;
    amp::Path2D path;
    int itr = 0;
    for (const auto& [n, r] : nr_pairs) {
        std::cout << "iteration: " << itr << std::endl;
        itr++;

        planner.prmSettings(n, r);
        std::vector<double> path_lengths_run;
        std::vector<double> computation_times_run;

        for (int i = 0; i < runs; ++i) {
            path = planner.plan(problem);
            computation_times_run.push_back(planner.duration);
            if (!path.waypoints.empty()){
                path_lengths_run.push_back(path.length());
            }
            else{
                path_lengths_run.push_back(0);
            }
        }

        std::string label = "n" + std::to_string(n) + "_r" + std::to_string(r);
        path_lengths[label] = path_lengths_run;
        computation_times[label] = computation_times_run;
    }

    // Convert maps to lists for the boxplot function
    std::list<std::vector<double>> path_length_data;
    std::list<std::vector<double>> computation_time_data;
    std::vector<std::string> labels;

    for (const auto& [label, data] : path_lengths) {
        path_length_data.push_back(data);
        labels.push_back(label);
    }

    for (const auto& [label, data] : computation_times) {
        computation_time_data.push_back(data);
    }

    amp::Visualizer::makeBoxPlot(path_length_data, labels, "Path Lengths", "Configuration", "Length");
    amp::Visualizer::makeBoxPlot(computation_time_data, labels, "Computation Times", "Configuration", "Time (microseconds)");
}

void exercise1(){
    amp::Problem2D problem = amp::HW5::getWorkspace1();
    MyPRM2D prm;
    amp::Path2D path = prm.plan(problem);
    amp::HW7::check(path, problem);

    if (path.valid) std::cout << "Path Length: " << path.length() << std::endl;
    std::cout << "Duration: " << prm.duration*0.001 << " miliseconds" << std::endl;
    Visualizer::makeFigure(problem, path);
    prm.visualizePRM(problem);
    Visualizer::makeFigure(problem, path);

    // std::cout << "Benchmarking" << std::endl;
    // std::vector<std::pair<int, double>> nr_pairs = {{200, 0.5}, {200, 1}, {200, 1.5}, {200, 2},{500, 0.5}, {500, 1}, {500, 1.5}, {500, 2}};
    // runBenchmark(prm, problem, 10, nr_pairs);

    Visualizer::showFigures();

}

void exercise2(){
    // amp::Problem2D problem = amp::HW2::getWorkspace1();
    amp::Problem2D problem = amp::HW2::getWorkspace2();

    MyPRM2D prm;
    amp::Path2D path = prm.plan(problem);
    amp::HW7::check(path, problem);

    if (path.valid) std::cout << "Path Length: " << path.length() << std::endl;
    std::cout << "Duration: " << prm.duration*0.001 << " miliseconds" << std::endl;
    Visualizer::makeFigure(problem, path);
    prm.visualizePRM(problem);
    Visualizer::makeFigure(problem, path);

    // std::vector<std::pair<int, double>> nr_pairs = {{200, 1}, {200, 2}, {500, 1}, {500, 2}, {1000, 1}, {1000, 2}};
    // runBenchmark(prm, problem, 100, nr_pairs);

    Visualizer::showFigures();
    
}

void exercise3(){
    // amp::Problem2D problem = amp::HW5::getWorkspace1();
    // amp::Problem2D problem = amp::HW2::getWorkspace1();
    amp::Problem2D problem = amp::HW2::getWorkspace2();

    int n = 5000;
    double r = 0.5;
    double goalBias = 0.5;
    double epsilon = 0.25;

    MyRRT2D rrt(n, r, goalBias, epsilon);
    amp::Path2D path = rrt.plan(problem);
    amp::HW7::check(path, problem);

    if (path.valid) std::cout << "Path Length: " << path.length() << std::endl;
    std::cout << "Duration: " << rrt.duration*0.001 << " miliseconds" << std::endl;
    Visualizer::makeFigure(problem, path);
    Visualizer::makeFigure(problem, path);

    // std::vector<std::pair<int, double>> nr_pairs = {{200, 1}, {200, 2}, {500, 1}, {500, 2}, {1000, 1}, {1000, 2}};
    // runBenchmark(prm, problem, 100, nr_pairs);

    Visualizer::showFigures();
    
}


int main(int argc, char** argv){
// "I'm giving just you participation credit" - Peter 3:19 PM MST 11/1/2023
    // exercise1();
    exercise2();
    // exercise3();

    int n = 5000;
    double r = 0.5;
    double goalBias = 0.5;
    double epsilon = 0.25;

    amp::HW7::grade<MyPRM2D, MyRRT2D>(
        "asra8222@colorado.edu", argc, argv,
        std::make_tuple(), // No constructor arguments for MyPRM2D
        std::make_tuple(n, r, goalBias, epsilon) // Constructor arguments for MyRRT2D
    );
   
   return 0; 
}