#include "AMPCore.h"
#include "hw/HW7.h"
#include "hw/HW2.h"
#include "hw/HW8.h"
#include "MyCentralizedMultiAgentRRT.h"
#include "MyDecentralizedMultiAgentRRT.h"
#include "HelpfulClass.h"

using namespace amp;

// Function to calculate the average of a vector
template <typename T>
double calculateAverage(const std::vector<T>& vec) {
    return std::accumulate(vec.begin(), vec.end(), 0.0) / vec.size();
}

void problem1() {
    int max_iterations = 7500;
    double step_size = 0.5;
    double goal_bias_probability = 0.05;
    double goal_radius = 0.25;

    std::vector<int> numAgentsList = {2, 3, 4, 5, 6};
    std::vector<double> averageComputationTimes;
    std::vector<double> averageTreeSizes;
    std::vector<std::string> labelsForBarGraph;

    for (int numAgents : numAgentsList) {
        amp::MultiAgentProblem2D problem = amp::HW8::getWorkspace1(numAgents);
        MyCentralizedMultiAgentRRT multiRRT(max_iterations, step_size, goal_bias_probability, goal_radius);

        std::vector<double> computationTimes;
        std::vector<int> treeSizes;

        for (int run = 0; run < 100; ++run) {
            std::cout << "Agent: " << numAgents << ", run: " << run << std::endl;
            amp::MultiAgentPath2D path = multiRRT.plan(problem);
            auto [time, size] = multiRRT.getBenchmarkData();
            computationTimes.push_back(time);
            treeSizes.push_back(size);
        }

        // Compute averages
        double avgComputationTime = calculateAverage(computationTimes);
        double avgTreeSize = calculateAverage(treeSizes);
        averageComputationTimes.push_back(avgComputationTime);
        averageTreeSizes.push_back(avgTreeSize);
        labelsForBarGraph.push_back("m = " + std::to_string(numAgents));

        std::vector<double> treeSizesDouble(treeSizes.begin(), treeSizes.end());

        std::list<std::vector<double>> dataSets = {computationTimes, treeSizesDouble};
        std::string title = "Benchmark Results for " + std::to_string(numAgents) + " Agents";
        std::vector<std::string> boxPlotLabels = {"Computation Time (microseconds)", "Tree Size"};
        Visualizer::makeBoxPlot(dataSets, boxPlotLabels, title, "Categories", "Values");
    }

    // Create bar graphs
    Visualizer::makeBarGraph(averageComputationTimes, labelsForBarGraph, "Average Computation Time vs Number of Agents", "Number of Agents", "Average Computation Time (microseconds)");
    Visualizer::makeBarGraph(averageTreeSizes, labelsForBarGraph, "Average Tree Size vs Number of Agents", "Number of Agents", "Average Tree Size");
    Visualizer::showFigures();
}

void problem2() {
    int max_iterations = 7500;
    double step_size = 0.5;
    double goal_bias_probability = 0.05;
    double goal_radius = 0.25;
    
    std::vector<int> numAgentsList = {2, 3, 4, 5, 6};
    std::vector<double> averageComputationTimes;
    std::vector<std::string> labelsForBarGraph;

    for (int numAgents : numAgentsList) {
        amp::MultiAgentProblem2D problem = amp::HW8::getWorkspace1(numAgents);
        MyDecentralizedMultiAgentRRT multiRRT(max_iterations, step_size, goal_bias_probability, goal_radius);

        std::vector<double> computationTimes;

        for (int run = 0; run < 100; ++run) {
            std::cout << "Agent: " << numAgents << ", run: " << run << std::endl;
            amp::MultiAgentPath2D path = multiRRT.plan(problem);
            double time = multiRRT.getBenchmarkData(); // Assuming this function returns the computation time
            computationTimes.push_back(time);
        }

        // Compute average computation time
        double avgComputationTime = calculateAverage(computationTimes);
        averageComputationTimes.push_back(avgComputationTime);
        labelsForBarGraph.push_back("m = " + std::to_string(numAgents));

        // Boxplot for each set of agents
        std::list<std::vector<double>> dataSets = {computationTimes};
        std::string title = "Benchmark Results for " + std::to_string(numAgents) + " Agents";
        std::vector<std::string> boxPlotLabels = {"Computation Time (microseconds)"};
        Visualizer::makeBoxPlot(dataSets, boxPlotLabels, title, "Categories", "Values");
    }

    // Create bar graph for average computation times
    Visualizer::makeBarGraph(averageComputationTimes, labelsForBarGraph, "Average Computation Time vs Number of Agents", "Number of Agents", "Average Computation Time (microseconds)");
    Visualizer::showFigures();
}

int main(int argc, char** argv){

    // amp::MultiAgentProblem2D problem = amp::HW8::getWorkspace1(2);

    // int max_iterations = 7500;
    // double step_size = 0.5;
    // double goal_bias_probability = 0.05;
    // double goal_radius = 0.25;

    // MyDecentralizedMultiAgentRRT multiRRT(max_iterations, step_size, goal_bias_probability, goal_radius);

    // amp::MultiAgentPath2D path = multiRRT.plan(problem);
    // std::vector<std::vector<Eigen::Vector2d>> collisionStates;

    // // Call the check function
    // bool isValid = amp::HW8::check(path, problem, collisionStates, true);
    // Visualizer::makeFigure(problem, path, collisionStates);
    // Visualizer::showFigures();

    // problem1();
    // problem2();

    int max_iterations = 7500;
    double step_size = 0.5;
    double goal_bias_probability = 0.05;
    double goal_radius = 0.25;

    MyCentralizedMultiAgentRRT centralizedRRT(max_iterations, step_size, goal_bias_probability, goal_radius);
    MyDecentralizedMultiAgentRRT decentralizedRRT(max_iterations, step_size, goal_bias_probability, goal_radius);

    amp::HW8::grade(centralizedRRT, decentralizedRRT, "asra8222@colorado.edu", argc, argv);

    return 0;
}

