#include "MyMultiAgentRRT.h"

MyCentralizedMultiAgentRRT::MyCentralizedMultiAgentRRT(int max_iterations, double step_size, double goal_bias_probability, double goal_radius)
    : n(max_iterations), r(step_size), goalBias(goal_bias_probability), epsilon(goal_radius) {}

amp::MultiAgentPath2D MyCentralizedMultiAgentRRT::plan(const amp::MultiAgentProblem2D& problem) {
    auto start_time = std::chrono::high_resolution_clock::now();

    // Initialize the trees and parent indices for each agent based on the initial positions
    trees.clear();
    parents.clear();
    trees.resize(problem.agent_properties.size());
    parents.resize(problem.agent_properties.size());

    for (size_t i = 0; i < problem.agent_properties.size(); ++i) {
        trees[i].push_back(problem.agent_properties[i].q_init);
        parents[i].push_back(-1); // Start state has no parent
    }

    // Set the environment and problem parameters
    xmin = problem.x_min;
    xmax = problem.x_max;
    ymin = problem.y_min;
    ymax = problem.y_max;
    obstacles = problem.obstacles;
    agent_properties = problem.agent_properties;

    std::vector<bool> goalReached(problem.agent_properties.size(), false);
    std::vector<int> goalIndices(problem.agent_properties.size(), -1);

    int iteration = 0;
    while (iteration < n) {
        // Sample free space for each agent
        auto samples = sampleFree();

        // Find the nearest node in the tree for each sample
        auto nearestIndices = findNearest(samples);

        // Steer from nearest towards the sample for each agent
        auto newVertices = steer(samples, nearestIndices);

        // Check if the path from nearest to new vertex is valid
        for (size_t i = 0; i < newVertices.size(); ++i) {
            if (isPathValid(trees[i][nearestIndices[i]], newVertices[i], problem, agent_properties[i].radius)) {
                // Add the new vertex to the tree and update the parent
                trees[i].push_back(newVertices[i]);
                parents[i].push_back(nearestIndices[i]);

                // Check if the new vertex is close to the goal
                if ((newVertices[i] - agent_properties[i].q_goal).norm() <= epsilon) {
                    goalReached[i] = true;
                    goalIndices[i] = trees[i].size() - 1;
                }
            }
        }

        // If all agents have reached the goal, break the loop
        if (std::all_of(goalReached.begin(), goalReached.end(), [](bool reached) { return reached; })) {
            break;
        }

        iteration++;
    }

    // Reconstruct paths from the goal index for each agent
    auto paths = reconstructPaths(goalIndices);

    // Convert the paths to the MultiAgentPath2D format
    amp::MultiAgentPath2D multiAgentPath;
    multiAgentPath.valid = std::all_of(goalReached.begin(), goalReached.end(), [](bool reached) { return reached; });
    multiAgentPath.paths.resize(paths.size());
    for (size_t i = 0; i < paths.size(); ++i) {
        for (int index : paths[i]) {
            multiAgentPath.paths[i].waypoints.push_back(trees[i][index]);
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

    return multiAgentPath;
}

std::vector<Eigen::Vector2d> MyCentralizedMultiAgentRRT::sampleFree() const {
    std::vector<Eigen::Vector2d> samples;
    samples.reserve(agent_properties.size());

    amp::RNG::seed(std::chrono::high_resolution_clock::now().time_since_epoch().count());
    
    tools tools;
    int iterationsMax = 1000;

    // Loop over each agent to get a sample for each one
    for (const auto& agent : agent_properties) {
        Eigen::Vector2d sample;
        bool validSample = false;
        int iterations = 0;

        // Keep sampling until a valid sample is found or we hit the iteration limit
        while (!validSample && iterations < iterationsMax) {
            // Sample the goal with a probability of goalBias, else sample randomly
            if (amp::RNG::srandd(0, 1) < goalBias) {
                sample = agent.q_goal;
            } else {
                double x = amp::RNG::srandd(xmin, xmax);
                double y = amp::RNG::srandd(ymin, ymax);
                sample = Eigen::Vector2d(x, y);
            }

            // Check if the sample is in any obstacle
            bool insideAnyObstacle = false;
            for (const amp::Obstacle2D& obstacle : obstacles) {
                if (tools.isPointInsidePolygon(sample, obstacle.verticesCCW())) {
                    insideAnyObstacle = true;
                    break;
                }
            }

            // If the sample is not in an obstacle, it's valid
            if (!insideAnyObstacle) {
                validSample = true;
            }

            iterations++;
        }

        // If we couldn't find a valid sample, throw an exception
        if (!validSample) {
            throw std::runtime_error("Failed to find a valid sample for agent after " + std::to_string(iterationsMax) + " iterations.");
        }

        // Add the valid sample to the list of samples
        samples.push_back(sample);
    }

    return samples;
}

std::vector<int> MyCentralizedMultiAgentRRT::findNearest(const std::vector<Eigen::Vector2d>& samples) const {
    std::vector<int> nearestIndices(samples.size(), -1);
    std::vector<double> nearestDists(samples.size(), std::numeric_limits<double>::max());

    // Assuming 'trees' is a vector of vectors, where each sub-vector is the RRT tree for a particular agent
    for (size_t agentIndex = 0; agentIndex < samples.size(); ++agentIndex) {
        const auto& sample = samples[agentIndex];

        for (size_t nodeIndex = 0; nodeIndex < trees[agentIndex].size(); ++nodeIndex) {
            double dist = (sample - trees[agentIndex][nodeIndex]).squaredNorm(); // Use squared distance to avoid sqrt calculation
            if (dist < nearestDists[agentIndex]) {
                nearestDists[agentIndex] = dist;
                nearestIndices[agentIndex] = nodeIndex;
            }
        }
    }

    return nearestIndices;
}

std::vector<Eigen::Vector2d> MyCentralizedMultiAgentRRT::steer(const std::vector<Eigen::Vector2d>& nearest, const std::vector<Eigen::Vector2d>& sample) const {
    std::vector<Eigen::Vector2d> newPositions;
    newPositions.reserve(nearest.size());  // Reserve space for efficiency

    // Steer towards each sample from the corresponding nearest position
    for (size_t i = 0; i < nearest.size(); ++i) {
        // Calculate the direction vector from nearest to sample
        Eigen::Vector2d direction = (sample[i] - nearest[i]).normalized();
        // Move from nearest to sample, but at most a distance of stepSize
        newPositions.push_back(nearest[i] + direction * std::min((sample[i] - nearest[i]).norm(), r));
    }

    return newPositions;
}

bool MyCentralizedMultiAgentRRT::checkCollisionBetweenRobots(const std::vector<Eigen::Vector2d>& positions, double radius) const {
    for (size_t i = 0; i < positions.size(); ++i) {
        for (size_t j = i + 1; j < positions.size(); ++j) {
            if ((positions[i] - positions[j]).norm() < 2 * radius) {
                return true; // Collision detected
            }
        }
    }
    return false; // No collision
}

bool MyCentralizedMultiAgentRRT::isPathValid(const std::vector<Eigen::Vector2d>& start, const std::vector<Eigen::Vector2d>& end, const MultiAgentProblem2D& problem, double robotRadius) const {
    // Check for collisions between robots
    if (checkCollisionBetweenRobots(start, robotRadius) || checkCollisionBetweenRobots(end, robotRadius)) {
        return false;
    }

    // Check for collisions with obstacles for each robot
    for (size_t i = 0; i < start.size(); ++i) {
        // Here we are reusing the logic from the single-agent RRT.
        // The tools object and the obstacles are assumed to be accessible in this context,
        // similar to how they are used in MyRRT2D::isPathValid.
        for (const auto& obstacle : problem.obstacles) {
            // Get the vertices of the obstacle
            const auto& vertices = obstacle.verticesCCW();
            size_t numVertices = vertices.size();

            // Check for intersection with each edge of the obstacle
            for (size_t j = 0; j < numVertices; ++j) {
                Eigen::Vector2d p1 = vertices[j];
                Eigen::Vector2d q1 = vertices[(j + 1) % numVertices]; // Loop back to the first vertex

                if (tools.isIntersecting(start[i], end[i], p1, q1)) {
                    return false; // The path for robot i intersects with an obstacle edge
                }
            }
        }
    }

    return true; // No intersections found, path is valid
}


std::vector<std::vector<int>> MyCentralizedMultiAgentRRT::reconstructPaths(const std::vector<int>& goalIndices) const {
    std::vector<std::vector<int>> paths;
    paths.reserve(goalIndices.size());  // Reserve space for efficiency

    // Iterate over all goal indices to reconstruct the path for each agent
    for (int goalIndex : goalIndices) {
        std::vector<int> path;
        while (goalIndex != -1) {
            // Add the current index to the path
            path.push_back(goalIndex);
            // Move to the parent of the current index
            goalIndex = parents[goalIndex];
        }
        // Reverse the path to start from the root
        std::reverse(path.begin(), path.end());
        // Add the reconstructed path to the paths vector
        paths.push_back(path);
    }

    return paths;
}
