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
        std::vector<Eigen::Vector2d> nearestPoints;
        for (size_t i = 0; i < samples.size(); ++i) {  
            nearestPoints.push_back(trees[i][nearestIndices[i]]);
        }
        auto newVertices = steer(nearestPoints, samples);

        // Check if the path from nearest to new vertex is valid for each agent
        // Initialize startSegment and endSegment for all agents
        std::vector<Eigen::Vector2d> startSegment, endSegment;
        for (size_t i = 0; i < newVertices.size(); ++i) {
            startSegment.push_back(trees[i][nearestIndices[i]]);
            endSegment.push_back(newVertices[i]);
        }
        // std::cout << "Size of start vector: " << startSegment.size() << std::endl;
        // std::cout << "Size of end vector: " << endSegment.size() << std::endl;
        if (isPathValid(startSegment, endSegment)) {
            for (size_t i = 0; i < newVertices.size(); ++i) {
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
            std::cout << "Goal reached for all agents" << std::endl;
            break;
        }

        iteration++;
    }

    // Reconstruct paths from the goal index for each agent
    auto paths = reconstructPaths(goalIndices);

    // Convert the paths to the MultiAgentPath2D format
    amp::MultiAgentPath2D multiAgentPath;
    multiAgentPath.valid = std::all_of(goalReached.begin(), goalReached.end(), [](bool reached) { return reached; });
    multiAgentPath.agent_paths.resize(paths.size());
    for (size_t i = 0; i < paths.size(); ++i) {
        for (int index : paths[i]) {
            multiAgentPath.agent_paths[i].waypoints.push_back(trees[i][index]);
        }
        // Add the goal position as the last waypoint
        multiAgentPath.agent_paths[i].waypoints.push_back(agent_properties[i].q_goal);
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
    int iterationsMax = 5000;

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

            bool insideObstacle = false;
            for (const amp::Obstacle2D& obstacle : obstacles) {
                if (tools.isPointInsidePolygon(sample, obstacle.verticesCCW())) {
                    insideObstacle = true;
                    break;
                }
            }

            if (!insideObstacle) {
                // Check the distance from all obstacles
                validSample = true;  // Assume the sample is valid initially
                for (const amp::Obstacle2D& obstacle : obstacles) {
                    Eigen::Vector2d closestPointVector = tools.distanceObstacle(sample, obstacle);
                    if (closestPointVector.norm() <= agent.radius) {
                        validSample = false;  // Sample is invalid if too close to an obstacle
                        break;
                    }
                    // else{
                    //     std::cout << "agent radius: " << agent.radius << ", distance from obstacle: " << closestPointVector.norm() << std::endl;
                    // }
                }
            }

            iterations++;
        }

        // Add the valid sample to the list of samples if it's valid
        if (validSample) {
            samples.push_back(sample);
        }
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
    newPositions.reserve(nearest.size());

    for (size_t i = 0; i < nearest.size(); ++i) {
        Eigen::Vector2d direction = (sample[i] - nearest[i]).normalized();
        newPositions.push_back(nearest[i] + direction * std::min((sample[i] - nearest[i]).norm(), r));
    }

    return newPositions;
}

bool MyCentralizedMultiAgentRRT::checkCollisionBetweenRobots(const std::vector<Eigen::Vector2d>& start, const std::vector<Eigen::Vector2d>& end) const {
    tools tools;

    for (size_t i = 0; i < start.size(); ++i) {
        for (size_t j = i + 1; j < end.size(); ++j) {
            double combinedRadius = agent_properties[i].radius + agent_properties[j].radius;

            // Calculate the minimum distance between the segments
            double minDist = tools.minDistanceBetweenSegments(start[i], end[i], start[j], end[j]);

            if (minDist <= combinedRadius) {
                return true; // Collision detected
            }
        }
    }
    return false; // No collision
}

bool MyCentralizedMultiAgentRRT::isPathValid(const std::vector<Eigen::Vector2d>& start, const std::vector<Eigen::Vector2d>& end) const {
    tools tools;

    // Check for collisions with obstacles for each robot
    for (size_t i = 0; i < start.size(); ++i) {
        double radius = agent_properties[i].radius;
        Eigen::Vector2d direction = (end[i] - start[i]).normalized();
        double segmentLength = (end[i] - start[i]).norm();

        for (double t = 0; t <= segmentLength; t += 0.01 * r) {  // Iterate along the segment
            Eigen::Vector2d point = start[i] + direction * t;  // Current point along the segment

            // Obstacle collision check
            for (const auto& obstacle : obstacles) {
                if (tools.distanceObstacle(point, obstacle).norm() <= radius) {
                    return false;  // Collision with obstacle detected
                }
            }

            // Robot-robot collision check at the current point
            std::vector<Eigen::Vector2d> currentPositions(start.size());
            for (size_t j = 0; j < start.size(); ++j) {
                Eigen::Vector2d otherDirection = (end[j] - start[j]).normalized();
                double otherSegmentLength = (end[j] - start[j]).norm();
                double otherT = std::min(t, otherSegmentLength);
                currentPositions[j] = start[j] + otherDirection * otherT;
            }

            if (checkCollisionBetweenRobots(start, currentPositions)) {
                return false;  // Collision between robots detected
            }
            if (!checkCollisionBetweenRobots(start, currentPositions)) {
                // Collision-free path found, log details
                for (size_t k = 0; k < start.size(); ++k) {
                    std::cout << "  Agent " << k << " Path: " << start[k].transpose() << " to " << currentPositions[k].transpose() << std::endl;
                }
            }
        }
    }

    return true; // No collision detected
}

std::vector<std::vector<int>> MyCentralizedMultiAgentRRT::reconstructPaths(const std::vector<int>& goalIndices) const {
    std::vector<std::vector<int>> paths;
    paths.reserve(goalIndices.size());  // Reserve space for efficiency

    // Iterate over all goal indices to reconstruct the path for each agent
    for (size_t agentIndex = 0; agentIndex < goalIndices.size(); ++agentIndex) {
        int goalIndex = goalIndices[agentIndex];
        std::vector<int> path;
        while (goalIndex != -1) {
            // Add the current index to the path
            path.push_back(goalIndex);
            // Move to the parent of the current index
            goalIndex = parents[agentIndex][goalIndex];
        }
        // Reverse the path to start from the root
        std::reverse(path.begin(), path.end());
        // Add the reconstructed path to the paths vector
        paths.push_back(path);
    }

    return paths;
}

