#include "MyRRT.h"

MyRRT2D::MyRRT2D(int max_iterations, double step_size, double goal_bias_probability, double goal_radius)
    : n(max_iterations), r(step_size), goalBias(goal_bias_probability), epsilon(goal_radius) {}

amp::Path MyRRT2D::plan(const Eigen::VectorXd& q_start,
                   const Eigen::VectorXd& q_goal) {
    // Initialize the tree with the start state
    tree.clear();
    parents.clear();
    tree.push_back(q_start.head<2>());
    parents.push_back(-1); // Start state has no parent

    int counter = 0;
    bool goalReached = false;
    int goalIndex = -1;

    while (counter < n) {
        Eigen::Vector2d sample = sampleFree(q_goal);

        int nearestIndex = findNearest(sample);
        Eigen::Vector2d newVertex = steer(tree[nearestIndex], sample);

        if (isPathValid(tree[nearestIndex], newVertex)) {
            tree.push_back(newVertex);
            parents.push_back(nearestIndex);

            if ((newVertex - q_goal.head<2>()).norm() <= epsilon) {
                goalReached = true;
                goalIndex = tree.size() - 1;
                break; // Exit the loop if the goal is reached
            }
        }

        counter++;
    }

    amp::Path path;
    path.valid = goalReached;
    if (goalReached) {
        std::vector<int> node_path_indices = reconstructPath(goalIndex);

        for (int idx : node_path_indices) {
            path.waypoints.push_back(tree[idx]);
        }
        if (!path.waypoints.back().isApprox(q_goal.head<2>())) {
            path.waypoints.push_back(q_goal.head<2>());
        }
    }
    return path;
}

Eigen::Vector2d MyRRT2D::sampleFree(const Eigen::VectorXd& q_goal) const {
    
    amp::RNG::seed(std::chrono::high_resolution_clock::now().time_since_epoch().count());
    
    tools tools;
    int iterations = 0;
    int iterationsMax = 1000;
    Eigen::Vector2d sample;

    while (iterations < iterationsMax) {
        // Generate a random sample with goal bias
        if (amp::RNG::srandd(0, 1) < goalBias) {
            sample = q_goal.head<2>();
        } else {
            double x = amp::RNG::srandd(xmin, xmax);
            double y = amp::RNG::srandd(ymin, ymax);
            sample = Eigen::Vector2d(x, y);
        }
        
        // Check if the sample is in an obstacle
        bool insideAnyObstacle = false;
        for (const amp::Obstacle2D& obstacle : obstacles) {
            if (tools.isPointInsidePolygon(sample, obstacle.verticesCCW())) {
                insideAnyObstacle = true;
                break;
            }
        }

        if (!insideAnyObstacle) {
            return sample; // Return the valid sample
        }

        iterations++;
    }

    throw std::runtime_error("Failed to find a valid sample after " + std::to_string(iterationsMax) + " iterations.");
}

int MyRRT2D::findNearest(const Eigen::Vector2d& sample) const {
    int nearestIndex = -1;
    double nearestDist = std::numeric_limits<double>::max();

    for (size_t i = 0; i < tree.size(); ++i) {
        double dist = (sample - tree[i]).squaredNorm(); // Use squared distance to avoid sqrt calculation
        if (dist < nearestDist) {
            nearestDist = dist;
            nearestIndex = i;
        }
    }

    return nearestIndex;
}

Eigen::Vector2d MyRRT2D::steer(const Eigen::Vector2d& nearest, const Eigen::Vector2d& sample) const {
    Eigen::Vector2d direction = (sample - nearest).normalized();
    return nearest + direction * std::min((sample - nearest).norm(), r);
}

bool MyRRT2D::isPathValid(const Eigen::Vector2d& start, const Eigen::Vector2d& end) const {
    // Iterate through each obstacle
    tools tools;
    for (const auto& obstacle : obstacles) {
        // Get the vertices of the obstacle
        const auto& vertices = obstacle.verticesCCW();
        size_t numVertices = vertices.size();

        // Check for intersection with each edge of the obstacle
        for (size_t i = 0; i < numVertices; ++i) {
            Eigen::Vector2d p1 = vertices[i];
            Eigen::Vector2d q1 = vertices[(i + 1) % numVertices]; // Loop back to the first vertex

            if (tools.isIntersecting(start, end, p1, q1)) {
                return false; // The path intersects with an obstacle edge
            }
        }
    }
    return true; // No intersections found, path is valid
}

std::vector<int> MyRRT2D::reconstructPath(int nodeIndex) const {
    std::vector<int> path;
    while (nodeIndex != -1) {
        path.push_back(nodeIndex);
        nodeIndex = parents[nodeIndex];
    }
    std::reverse(path.begin(), path.end()); // Reverse the path to start from the root
    return path;
}
