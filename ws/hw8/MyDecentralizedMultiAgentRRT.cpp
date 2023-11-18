#include "MyDecentralizedMultiAgentRRT.h"


MyDecentralizedMultiAgentRRT::MyDecentralizedMultiAgentRRT(int max_iterations, double step_size, double goal_bias_probability, double goal_radius)
        : n(max_iterations), r(step_size), goalBias(goal_bias_probability), epsilon(goal_radius) {}

amp::MultiAgentPath2D MyDecentralizedMultiAgentRRT::plan(const amp::MultiAgentProblem2D& problem) {
    auto start_time = std::chrono::high_resolution_clock::now();

    // Environment setup
    xmin = problem.x_min;
    xmax = problem.x_max;
    ymin = problem.y_min;
    ymax = problem.y_max;
    obstacles = problem.obstacles;
    agent_properties = problem.agent_properties;

    amp::MultiAgentPath2D multiAgentPath;
    multiAgentPath.agent_paths.resize(agent_properties.size());

    trees.resize(agent_properties.size());
    parents.resize(agent_properties.size());
    dynamicObstacles.clear();
    timeIndices.resize(agent_properties.size());

    for (size_t agentIndex = 0; agentIndex < agent_properties.size(); ++agentIndex) {
        trees[agentIndex].clear();
        parents[agentIndex].clear();
        timeIndices[agentIndex].clear(); 

        trees[agentIndex].emplace_back(agent_properties[agentIndex].q_init);
        parents[agentIndex].emplace_back(-1);
        timeIndices[agentIndex].emplace_back(0);

        bool goalReached = false;
        int iteration = 0;
        while (iteration < n && !goalReached) {
            Eigen::Vector2d sample = sampleFree(agentIndex, iteration);
            int nearestIndex = findNearest(agentIndex, sample);
            Eigen::Vector2d newVertex = steer(trees[agentIndex][nearestIndex], sample);

            int newNodeTimeIndex = timeIndices[agentIndex][nearestIndex] + 1; // Increment time index

            if (isPathValid(trees[agentIndex][nearestIndex], newVertex, agent_properties[agentIndex].radius, newNodeTimeIndex)) {
                trees[agentIndex].push_back(newVertex);
                parents[agentIndex].push_back(nearestIndex);
                timeIndices[agentIndex].push_back(newNodeTimeIndex);

                if ((newVertex - agent_properties[agentIndex].q_goal).norm() <= epsilon) {
                    auto path = reconstructPath(agentIndex);
                    if (postPathConstructionCheck(path, agent_properties[agentIndex].radius)) {
                        goalReached = true;
                        for (const auto& waypoint : path) {
                            multiAgentPath.agent_paths[agentIndex].waypoints.push_back(waypoint);
                        }
                        multiAgentPath.agent_paths[agentIndex].valid = true;
                    }
                }
            }
            iteration++;
        }

        if (!goalReached) {
            multiAgentPath.agent_paths[agentIndex].valid = false;
        }
        else {
            // Add the valid path of the current agent to dynamicObstacles for future collision checks
            dynamicObstacles.push_back(multiAgentPath.agent_paths[agentIndex].waypoints);
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

    return multiAgentPath;
}

Eigen::Vector2d MyDecentralizedMultiAgentRRT::sampleFree(size_t agentIndex, int iteration) const {
    Eigen::Vector2d sample;
    amp::RNG::seed(std::chrono::high_resolution_clock::now().time_since_epoch().count());

    tools tools;
    int iterationsMax = 5000;
    int goalBiasFrequency = 20;
    bool forceGoalSampling = (iteration % goalBiasFrequency == 0);
    int iterations = 0;

    amp::CircularAgentProperties currentAgent = agent_properties[agentIndex];

    bool validSample = false;
    while (iterations < iterationsMax) {
        // Sample the goal with a probability of goalBias, else sample randomly
        if (forceGoalSampling || amp::RNG::srandd(0, 1) < goalBias) {
            sample = currentAgent.q_goal;
        } else {
            double x = amp::RNG::srandd(xmin, xmax);
            double y = amp::RNG::srandd(ymin, ymax);
            sample = Eigen::Vector2d(x, y);
        }

        // Check if the sample is in a static obstacle
        bool inStaticObstacle = false;
        for (const auto& obstacle : obstacles) {
            if (tools.isPointInsidePolygon(sample, obstacle.verticesCCW())) {
                inStaticObstacle = true;
                break;
            }
        }
        if (!inStaticObstacle) {
            // Check the distance from all obstacles
            validSample = true;  // Assume the sample is valid initially
            for (const amp::Obstacle2D& obstacle : obstacles) {
                Eigen::Vector2d closestPointVector = tools.distanceObstacle(sample, obstacle);
                if (closestPointVector.norm() <= currentAgent.radius) {
                    validSample = false;  // Sample is invalid if too close to an obstacle
                    break;
                }
            }
        }

        // // Check if the sample is near any dynamic obstacle (paths of other agents)
        // bool inDynamicObstacle = false;
        // for (size_t j = 0; j < dynamicObstacles.size(); ++j) {
        //     for (size_t i = 0; i < dynamicObstacles[j].size() - 1; ++i) {
        //         Eigen::Vector2d closestPoint = tools.closestPointOnSegment(sample, dynamicObstacles[j][i], dynamicObstacles[j][i+1]);
        //         double combinedRadius = currentAgent.radius + agent_properties[j].radius; // Assuming agent_properties is accessible
        //         if ((closestPoint - sample).norm() <= combinedRadius) {
        //             inDynamicObstacle = true;
        //             break;
        //         }
        //     }
        //     if (inDynamicObstacle) {
        //         break;
        //     }
        // }

        if (validSample) {
            return sample; // Return the valid sample
        }

        iterations++;
    }

    throw std::runtime_error("Failed to find a valid sample after " + std::to_string(iterationsMax) + " iterations.");
}

int MyDecentralizedMultiAgentRRT::findNearest(size_t agentIndex, const Eigen::Vector2d& sample) const {
    if (agentIndex >= trees.size()) {
        throw std::runtime_error("Agent index out of range in findNearest method.");
    }

    const auto& agentTree = trees[agentIndex];
    int nearestIndex = -1;
    double nearestDist = std::numeric_limits<double>::max();

    // Iterating over the RRT tree of the specified agent
    for (size_t i = 0; i < agentTree.size(); ++i) {
        double dist = (sample - agentTree[i]).squaredNorm(); // Use squared distance to avoid sqrt calculation
        if (dist < nearestDist) {
            nearestDist = dist;
            nearestIndex = i;
        }
    }

    return nearestIndex;
}

Eigen::Vector2d MyDecentralizedMultiAgentRRT::steer(const Eigen::Vector2d& nearest, const Eigen::Vector2d& sample) const {
    Eigen::Vector2d direction = (sample - nearest).normalized();
    return nearest + direction * std::min((sample - nearest).norm(), r);
}

bool MyDecentralizedMultiAgentRRT::isPathValid(const Eigen::Vector2d& start, const Eigen::Vector2d& end, double agentRadius, int timeIndex) const {
    tools tools;

    double parameterization = 0.01;

    Eigen::Vector2d direction = (end - start).normalized();
    double segmentLength = (end - start).norm() + agentRadius;

    for (double t = 0; t <= segmentLength; t += parameterization * r) {
        Eigen::Vector2d point_i = start + direction * t;  // Current point along the segment

        // Obstacle collision 
        for (const auto& obstacle : obstacles) {
            if (tools.distanceObstacle(point_i, obstacle).norm() <= agentRadius) {
                return false;  // Collision with obstacle detected
            }
        }

        // Check against higher priority paths
        for (size_t i = 0; i < dynamicObstacles.size(); ++i) {
            const auto& dynamicPath = dynamicObstacles[i];
            Eigen::Vector2d obstaclePosition = (timeIndex < dynamicPath.size()) ? dynamicPath[timeIndex] : dynamicPath.back();
            double combinedRadius = agentRadius + agent_properties[i].radius; 

            if ((point_i - obstaclePosition).norm() <= combinedRadius) {
                return false; // Collision detected
            }
        }
    }
    return true; // No intersections found, path is valid
}

std::vector<Eigen::Vector2d> MyDecentralizedMultiAgentRRT::reconstructPath(size_t agentIndex) const {
    if (agentIndex >= trees.size()) {
        throw std::runtime_error("Agent index out of range in reconstructPath method.");
    }

    const auto& agentTree = trees[agentIndex];
    const auto& agentParents = parents[agentIndex];
    std::vector<Eigen::Vector2d> path;

    int nodeIndex = agentTree.size() - 1;  // Start from the last node (goal node) of the agent's tree

    while (nodeIndex != -1) {
        path.push_back(agentTree[nodeIndex]);
        nodeIndex = agentParents[nodeIndex];
    }

    std::reverse(path.begin(), path.end());  // Reverse to start from the root

    // Ensure the goal position is exact if it's within epsilon
    if ((path.back() - agent_properties[agentIndex].q_goal).norm() <= epsilon) {
        path.back() = agent_properties[agentIndex].q_goal;
    }

    return path;
}

bool MyDecentralizedMultiAgentRRT::postPathConstructionCheck(const std::vector<Eigen::Vector2d>& path, double currentAgentRadius) {
    for (size_t timeStep = 0; timeStep < path.size(); ++timeStep) {
        const Eigen::Vector2d& currentPosition = path[timeStep];

        for (size_t obstacleIndex = 0; obstacleIndex < dynamicObstacles.size(); ++obstacleIndex) {
            const auto& otherPath = dynamicObstacles[obstacleIndex];
            Eigen::Vector2d otherPosition = (timeStep < otherPath.size()) ? otherPath[timeStep] : otherPath.back();
            double combinedRadius = currentAgentRadius + agent_properties[obstacleIndex].radius;

            if ((currentPosition - otherPosition).norm() <= combinedRadius) {
                return false; // Collision detected
            }
        }
    }
    return true; // No collision detected
}





