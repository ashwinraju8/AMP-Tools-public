#include "MyCentralizedMultiAgentRRT.h"

///////////// Centralized Multi-Agent RRT /////////////

MyCentralizedMultiAgentRRT::MyCentralizedMultiAgentRRT(int max_iterations, double step_size, double goal_bias_probability, double goal_radius)
    : n(max_iterations), r(step_size), goalBias(goal_bias_probability), epsilon(goal_radius) {}

amp::MultiAgentPath2D MyCentralizedMultiAgentRRT::plan(const amp::MultiAgentProblem2D& problem) {
    auto start_time = std::chrono::high_resolution_clock::now();

    // Setting up the problem
    problemPlotting = problem;
    xmin = problem.x_min;
    xmax = problem.x_max;
    ymin = problem.y_min;
    ymax = problem.y_max;
    obstacles = problem.obstacles;
    agent_properties = problem.agent_properties;

    // Initialize the tree and parent index
    tree.clear();
    parents.clear();

    // Start state for all agents
    std::vector<Eigen::Vector2d> startState;
    for (const auto& agent : problem.agent_properties) {
        startState.push_back(agent.q_init);
    }
    tree.push_back(startState);
    parents.push_back(-1); // Start state has no parent

    // Main RRT loop
    bool goalReached = false;
    int iteration = 0;
    int validCounter = 0;
    while (iteration < n && !goalReached) {
        auto sample = sampleFree(iteration);
        int nearestIndex = findNearest(sample);
        auto newVertex = steer(tree[nearestIndex], sample);

        if (isPathValid(tree[nearestIndex], newVertex)) {
            validCounter += 1;
            tree.push_back(newVertex);
            parents.push_back(nearestIndex);

            // Check if all agents have reached their goals
            goalReached = true;
            for (size_t i = 0; i < agent_properties.size(); ++i) {
                if ((newVertex[i] - agent_properties[i].q_goal).norm() > epsilon) {
                    goalReached = false;
                    break;
                }
                else{
                    // std::cout << "Agent " << i << " reached its goal" << std::endl;
                }
            }
        }
        iteration++;
    }
    // std::cout << "Valid Paths: " << validCounter << std::endl;
    // Path reconstruction
    std::vector<std::vector<Eigen::Vector2d>> paths;
    if (goalReached) {
        // std::cout << "Goal State Reached " << std::endl;
        int goalIndex = tree.size() - 1; // Assuming last added vertex is the goal state
        paths = reconstructPaths(goalIndex);
    }
    else{
        std::cout << "Fail" << std::endl;
        return amp::MultiAgentPath2D(problem.numAgents());
    }

    // Convert the paths to the MultiAgentPath2D format
    amp::MultiAgentPath2D multiAgentPath;
    multiAgentPath.valid = goalReached;
    multiAgentPath.agent_paths.resize(paths.size());
    for (size_t agentIndex = 0; agentIndex < paths.size(); ++agentIndex) {
        for (const auto& waypoint : paths[agentIndex]) {
            multiAgentPath.agent_paths[agentIndex].waypoints.push_back(waypoint);
        }
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

    return multiAgentPath;
}

std::vector<Eigen::Vector2d> MyCentralizedMultiAgentRRT::sampleFree(int iteration) const {
    std::vector<Eigen::Vector2d> samples;
    std::vector<Eigen::Vector2d> validSamples;  // Store valid samples for collision check
    samples.reserve(agent_properties.size());
    validSamples.reserve(agent_properties.size());
    amp::RNG::seed(std::chrono::high_resolution_clock::now().time_since_epoch().count());

    tools tools;
    int iterationsMax = 5000;

    int goalBiasFrequency = 20;
    bool forceGoalSampling = (iteration % goalBiasFrequency == 0);

    // Loop over each agent to get a sample for each one
    for (const auto& agent : agent_properties) {
        Eigen::Vector2d sample;
        bool validSample = false;
        int iterations = 0;

        // Keep sampling until a valid sample is found or we hit the iteration limit
        while (!validSample && iterations < iterationsMax) {
            // Sample the goal with a probability of goalBias, else sample randomly
            if (forceGoalSampling || amp::RNG::srandd(0, 1) < goalBias) {
                sample = agent.q_goal;
            } else {
                double x = amp::RNG::srandd(xmin, xmax);
                double y = amp::RNG::srandd(ymin, ymax);
                sample = Eigen::Vector2d(x, y);
            }

            // Check if the point is inside any obstacle
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
                }

                // Check for collision with other agents' samples
                if (validSample) {
                    for (const auto& otherSample : validSamples) {
                        double combinedRadius = agent.radius + agent_properties[&otherSample - &validSamples[0]].radius;
                        if ((sample - otherSample).norm() <= combinedRadius) {
                            validSample = false;
                            break;
                        }
                    }
                }
            }

            iterations++;
        }

        // Add the valid sample to the list of samples if it's valid
        if (validSample) {
            samples.push_back(sample);
            validSamples.push_back(sample);  // Add to the list of valid samples for collision checks
        }
    }

    return samples;
}

int MyCentralizedMultiAgentRRT::findNearest(const std::vector<Eigen::Vector2d>& sample) const {
    int nearestIndex = -1;
    double nearestDist = std::numeric_limits<double>::max();

    for (size_t nodeIndex = 0; nodeIndex < tree.size(); ++nodeIndex) {
        double dist = 0.0;
        for (size_t agentIndex = 0; agentIndex < sample.size(); ++agentIndex) {
            dist += (sample[agentIndex] - tree[nodeIndex][agentIndex]).squaredNorm();
        }

        if (dist < nearestDist) {
            nearestDist = dist;
            nearestIndex = nodeIndex;
        }
    }

    return nearestIndex;
}

std::vector<Eigen::Vector2d> MyCentralizedMultiAgentRRT::steer(const std::vector<Eigen::Vector2d>& nearest, const std::vector<Eigen::Vector2d>& sample) const {
    std::vector<Eigen::Vector2d> newPositions;
    newPositions.reserve(nearest.size());

    for (size_t i = 0; i < nearest.size(); ++i) {
        Eigen::Vector2d direction = (sample[i] - nearest[i]).normalized();
        double stepDistance = std::min((sample[i] - nearest[i]).norm(), r);
        newPositions.push_back(nearest[i] + direction * stepDistance);
    }

    return newPositions;
}

bool MyCentralizedMultiAgentRRT::isPathValid(const std::vector<Eigen::Vector2d>& start, const std::vector<Eigen::Vector2d>& end) const {
    tools tools;

    double buffer = 1.1;
    double parameterization = 0.001;

    // Vector to store the start and end points of each robot's path segment
    std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> pathSegments;

    // Iterate over each segment for each robot
    for (size_t i = 0; i < start.size(); ++i) {
        double radius = agent_properties[i].radius;
        Eigen::Vector2d direction = (end[i] - start[i]).normalized();
        double segmentLength = (end[i] - start[i]).norm() + radius;

        for (double t = 0; t <= segmentLength; t += parameterization * r) {
            Eigen::Vector2d point_i = start[i] + direction * t;  // Current point along the segment for robot i

            // Obstacle collision check for robot i
            for (const auto& obstacle : obstacles) {
                if (tools.distanceObstacle(point_i, obstacle).norm() <= radius) {
                    return false;  // Collision with obstacle detected
                }
            }

            // Robot-robot collision check at the current point for robot i
            for (size_t j = 0; j < start.size(); ++j) {
                if (i != j) { // Avoid checking collision with itself
                    Eigen::Vector2d otherDirection = (end[j] - start[j]).normalized();
                    Eigen::Vector2d point_j = start[j] + otherDirection*t; // Current point along the segment for robot j

                    double combinedRadius = radius + agent_properties[j].radius;

                    // Check collision with end[j] and start[j]
                    if ((point_i - end[j]).norm() <= buffer * combinedRadius || (start[i] - end[j]).norm() <= buffer * combinedRadius || (end[i] - start[j]).norm() <= buffer * combinedRadius) {
                        return false;  // Collision detected
                    }

                    if ((point_i - point_j).norm() <= buffer*combinedRadius) {
                        return false;  // Collision between robots i and j detected
                    }
                }
            }
        }
        // Store the segment that passed the collision check
        pathSegments.push_back({start[i], end[i]});
    }
    // Visualization part
    // amp::MultiAgentPath2D multiAgentPath(start.size());
    // for (size_t i = 0; i < start.size(); ++i) {
    //     // Add the start and end points for each agent
    //     multiAgentPath.agent_paths[i].waypoints.push_back(start[i]);
    //     multiAgentPath.agent_paths[i].waypoints.push_back(end[i]);
    // }

 
    // amp::MultiAgentProblem2D prob = problemPlotting; 

    
    // amp::Visualizer::makeFigure(prob, multiAgentPath);

    
    // amp::Visualizer::showFigures();


    return true; // No collision detected
}

std::vector<std::vector<Eigen::Vector2d>> MyCentralizedMultiAgentRRT::reconstructPaths(int goalIndex) const {
    std::vector<std::vector<Eigen::Vector2d>> paths(agent_properties.size());
    
    // Ensure the goal state is included
    // std::cout << "Reconstructing paths from goalIndex: " << goalIndex << std::endl;
    if (goalIndex >= 0 && goalIndex < tree.size()) {
        for (size_t agentIndex = 0; agentIndex < agent_properties.size(); ++agentIndex) {
            // std::cout << "Agent " << agentIndex << " Goal State: " << tree[goalIndex][agentIndex].transpose() << std::endl;
            paths[agentIndex].push_back(tree[goalIndex][agentIndex]);
        }
    }
    

    // Traverse back from the goal index to the start
    while (goalIndex != -1) {
        goalIndex = parents[goalIndex];
        if (goalIndex != -1) {
            for (size_t agentIndex = 0; agentIndex < agent_properties.size(); ++agentIndex) {
                paths[agentIndex].push_back(tree[goalIndex][agentIndex]);
            }
        }
    }

    // Add the start state (q_init) if not already included
    for (size_t agentIndex = 0; agentIndex < agent_properties.size(); ++agentIndex) {
        if (!paths[agentIndex].empty() && paths[agentIndex].back() != agent_properties[agentIndex].q_init) {
            paths[agentIndex].push_back(agent_properties[agentIndex].q_init);
        }
    }

    // Reverse each path to start from the root
    for (auto& path : paths) {
        std::reverse(path.begin(), path.end());
    }

    // ensure exact goal position is added if it's within epsilon
    for (size_t agentIndex = 0; agentIndex < agent_properties.size(); ++agentIndex) {
        if ((paths[agentIndex].back() - agent_properties[agentIndex].q_goal).norm() <= epsilon) {
            paths[agentIndex].back() = agent_properties[agentIndex].q_goal;
        }
    }


    return paths;
}

