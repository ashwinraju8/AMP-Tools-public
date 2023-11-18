#include "AMPCore.h"
#include <chrono>
#include <Eigen/Dense>
#include <vector>
#include "hw/HW8.h"
#include "HelpfulClass.h"
#include <fstream>

using namespace amp;

class MyCentralizedMultiAgentRRT : public amp::CentralizedMultiAgentRRT {
public:
    // Default constructor
    MyCentralizedMultiAgentRRT() : MyCentralizedMultiAgentRRT(7500, 0.5, 0.05, 0.25) {}

    MyCentralizedMultiAgentRRT(int max_iterations, double step_size, double goal_bias_probability, double goal_radius);

    virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;

    std::pair<double, int> getBenchmarkData() const {
        return std::make_pair(duration, tree.size());
    }

private:
    // Helper methods
    std::vector<Eigen::Vector2d> sampleFree(int iteration) const;
    int findNearest(const std::vector<Eigen::Vector2d>& sample) const;
    std::vector<Eigen::Vector2d> steer(const std::vector<Eigen::Vector2d>& nearest, const std::vector<Eigen::Vector2d>& sample) const;
    bool isPathValid(const std::vector<Eigen::Vector2d>& start, const std::vector<Eigen::Vector2d>& end) const;
    std::vector<std::vector<Eigen::Vector2d>> reconstructPaths(int goalIndex) const;

    // RRT specific variables
    std::vector<std::vector<Eigen::Vector2d>> tree; // The RRT tree representing all agents
    std::vector<int> parents; // Parent indices for the vertices in the tree

    // Other member variables
    double duration; // Duration of the planning in microseconds
    double r; // Step size
    double goalBias; // Goal bias probability
    double epsilon; // Radius for the termination condition at the goal
    int n; // Maximum number of iterations

    // Environment variables
    double xmin, xmax, ymin, ymax;
    std::vector<amp::Obstacle2D> obstacles;
    std::vector<amp::CircularAgentProperties> agent_properties;
    amp::MultiAgentProblem2D problemPlotting;
};


