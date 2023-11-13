#include "AMPCore.h"
#include "hw/HW7.h"
#include "hw/HW2.h"
#include "hw/HW8.h"
#include "MyMultiAgentRRT.h"

using namespace amp;

void checkPathCollisions(const amp::MultiAgentPath2D& path, amp::MultiAgentProblem2D problem) {
    for (size_t step = 0; step < path.agent_paths[0].waypoints.size(); ++step) {
        for (size_t i = 0; i < path.agent_paths.size(); ++i) {
            for (size_t j = i + 1; j < path.agent_paths.size(); ++j) {
                double dist = (path.agent_paths[i].waypoints[step] - path.agent_paths[j].waypoints[step]).norm();
                double combinedRadius = problem.agent_properties[i].radius + problem.agent_properties[j].radius;

                if (dist <= combinedRadius) {
                    std::cout << "Collision detected at step " << step
                              << " between agents " << i << " and " << j
                              << " at positions " << path.agent_paths[i].waypoints[step].transpose()
                              << " and " << path.agent_paths[j].waypoints[step].transpose() << std::endl;
                }
            }
        }
    }
}

int main(){

    amp::MultiAgentProblem2D problem = amp::HW8::getWorkspace1();

    int max_iterations = 1000;
    double step_size = 0.5;
    double goal_bias_probability = 0.05;
    double goal_radius = 0.25;

    MyCentralizedMultiAgentRRT multiRRT(max_iterations, step_size, goal_bias_probability, goal_radius);

    amp::MultiAgentPath2D path = multiRRT.plan(problem);
    // To store the collision states
    std::vector<std::vector<Eigen::Vector2d>> collisionStates;

    // Call the check function
    bool isValid = amp::HW8::check(path, problem, collisionStates, true);

    // Now you can check if the path is valid and look at the collision states if not
    // if (isValid) {
    //     std::cout << "The path is valid!" << std::endl;
    // } else {
    //     std::cout << "The path is not valid. Here are the collision states:" << std::endl;
    //     for (const auto& agentCollisions : collisionStates) {
    //         for (const auto& collision : agentCollisions) {
    //             std::cout << "Collision at: " << collision.transpose() << std::endl;
    //         }
    //     }
    // }
    checkPathCollisions(path, problem);

    // Visualizer::makeFigure(problem, path);
    Visualizer::makeFigure(problem, path, collisionStates);
    Visualizer::showFigures();

    return 0;
}