#include "AMPCore.h"
#include <chrono>
#include <Eigen/Dense>
#include <vector>
#include "hw/HW8.h"
#include "HelpfulClass.h"
#include <fstream>

using namespace amp;


class MyDecentralizedMultiAgentRRT : public amp::DecentralizedMultiAgentRRT {
public:
    // Default constructor
    MyDecentralizedMultiAgentRRT() : MyDecentralizedMultiAgentRRT(7500, 0.5, 0.05, 0.25) {}

    MyDecentralizedMultiAgentRRT(int max_iterations, double step_size, double goal_bias_probability, double goal_radius);

    virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;

    double getBenchmarkData() const {
        return duration;
    }

private:
    // Helper methods 
    Eigen::Vector2d sampleFree(size_t agentIndex, int iteration) const;
    int findNearest(size_t agentIndex, const Eigen::Vector2d& sample) const;
    Eigen::Vector2d steer(const Eigen::Vector2d& nearest, const Eigen::Vector2d& sample) const;
    bool isPathValid(const Eigen::Vector2d& start, const Eigen::Vector2d& end, double agentRadius, int timeIndex) const;
    std::vector<Eigen::Vector2d> reconstructPath(size_t agentIndex) const;
    bool postPathConstructionCheck(const std::vector<Eigen::Vector2d>& path, double currentAgentRadius);

    // RRT specific variables
    std::vector<std::vector<Eigen::Vector2d>> trees; // The RRT trees for each agent
    std::vector<std::vector<int>> parents; // Parent indices for the vertices in the trees
    std::vector<std::vector<Eigen::Vector2d>> dynamicObstacles; // Paths of higher priority agents as obstacles
    std::vector<std::vector<int>> timeIndices; // Time indices for each agent's tree

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
};