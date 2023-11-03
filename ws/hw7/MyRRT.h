#include "AMPCore.h"
#include "hw/HW7.h"
#include "HelpfulClass.h"
#include <chrono>

class MyRRT2D : public amp::GoalBiasRRT2D {
public:
    MyRRT2D(int max_iterations, double step_size, double goal_bias_probability, double goal_radius);

    virtual amp::Path2D plan(const amp::Problem2D& problem) override{
        auto start = std::chrono::high_resolution_clock::now();


        // Set the grid parameters
        xmin = problem.x_min;
        xmax = problem.x_max;
        ymin = problem.y_min;
        ymax = problem.y_max;
        obstacles = problem.obstacles;

        // Call the planner
        amp::Path path = plan(problem.q_init, problem.q_goal);

        // Convert the ND path to a 2D path and return it
        amp::Path2D path_2d;
        path_2d.valid = path.valid; // Copy the validity flag

        if (path_2d.valid) {
            for (const auto& point : path.waypoints) {
                path_2d.waypoints.push_back(point.head<2>());
            }
        }   

        auto end = std::chrono::high_resolution_clock::now();
        duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

        return path_2d;
    }

    double duration; // Duration in seconds

private:
    amp::Path plan(const Eigen::VectorXd& q_start,
                   const Eigen::VectorXd& q_goal);
    
    Eigen::Vector2d sampleFree(const Eigen::VectorXd& q_goal) const;
    int findNearest(const Eigen::Vector2d& sample) const;
    Eigen::Vector2d steer(const Eigen::Vector2d& nearest, const Eigen::Vector2d& sample) const;
    bool isPathValid(const Eigen::Vector2d& start, const Eigen::Vector2d& end) const;
    std::vector<int> reconstructPath(int nodeIndex) const;

    // Workspace variables
    double xmin, xmax, ymin, ymax;
    std::vector<amp::Obstacle2D> obstacles;

    // RRT specific variables
    std::vector<Eigen::Vector2d> tree; // The RRT tree of vertices
    std::vector<int> parents; // The parent indices for the vertices in the tree

    double r; // The maximum distance to expand the tree in one step
    double goalBias; // The probability of sampling the goal position
    double epsilon; // Goal radius
    int n; // Number of samples
};