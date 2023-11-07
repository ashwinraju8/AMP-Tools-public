#include "AMPCore.h"
#include <chrono>
#include <Eigen/Dense>
#include <vector>
#include "hw/HW8.h"
#include "HelpfulClass.h"

class MyCentralizedMultiAgentRRT : public amp::CentralizedMultiAgentRRT {
public:
    MyCentralizedMultiAgentRRT(int max_iterations, double step_size, double goal_bias_probability, double goal_radius);

    virtual amp::MultiAgentPath2D plan(const amp::MultiAgentProblem2D& problem) override;

    virtual ~MyCentralizedMultiAgentRRT() {}

private:
    // Helper methods
    std::vector<Eigen::Vector2d> sampleFree() const;
    std::vector<int> findNearest(const std::vector<Eigen::Vector2d>& samples) const;
    std::vector<Eigen::Vector2d> steer(const std::vector<Eigen::Vector2d>& nearest, const std::vector<Eigen::Vector2d>& sample) const;
    bool checkCollisionBetweenRobots(const std::vector<Eigen::Vector2d>& positions, double radius) const;
    bool isPathValid(const std::vector<Eigen::Vector2d>& start, const std::vector<Eigen::Vector2d>& end, const MultiAgentProblem2D& problem, double robotRadius) const;
    std::vector<std::vector<int>> reconstructPaths(const std::vector<int>& goalIndices) const;

    // RRT specific variables for multi-agent
    std::vector<std::vector<Eigen::Vector2d>> trees; // The RRT trees for all agents
    std::vector<std::vector<int>> parents; // Parent indices for the vertices in the trees

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