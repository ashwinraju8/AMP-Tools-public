#include "MyPRM.h"

amp::Path MyPRM2D::plan(const Eigen::VectorXd& init_state,
                        const Eigen::VectorXd& goal_state,
                        int n,
                        int r,
                        const amp::GridCSpace2D& collision_checker)
{

    // Sample n random configurations
    sampleConfigurations(n, collision_checker);

    // Add the start and goal states to the nodes map
    amp::Node start_node = nodes.size();
    amp::Node goal_node = start_node + 1;
    nodes[start_node] = init_state;
    nodes[goal_node] = goal_state;

    // Add nodes to the graph and connect valid configurations to each other within radius r
    for (const auto& [current_node, current_state] : nodes) {
        for (const auto& [potential_neighbor_node, potential_neighbor_state] : nodes) {
            // Ensure not connecting node to itself and that potential neighbor is within r
            if (current_node != potential_neighbor_node && (current_state - potential_neighbor_state).norm() <= r) {
                // Check that path between current node and potential neighbor is valid
                if (isPathValid(current_state.head<2>(), potential_neighbor_state.head<2>(), collision_checker)) {
                    // Calculate the edge weight as euclidean distance between nodes
                    double edgeWeight = (current_state - potential_neighbor_state).norm(); // compute the edge weight
                    // Connect current node to potential neighbor in graph
                    graph.connect(current_node, potential_neighbor_node, edgeWeight);
                }
            }
        }
    }

    // Check that graph exists
    if (graph.nodes().empty()) {
        std::cout << "The graph is empty. Path not found." << std::endl;
        return amp::Path();

    }

    // Use A* to find a path from init_state to goal_state
    amp::ShortestPathProblem problemAStar;
    problemAStar.graph = std::make_shared<amp::Graph<double>>(graph);
    problemAStar.init_node = start_node;
    problemAStar.goal_node = goal_node;

    

    // Create a heuristic returning 0
    amp::SearchHeuristic heuristic;

    // Run A* search
    MyAStarAlgo astar;
    amp::AStar::GraphSearchResult result = astar.search(problemAStar, heuristic);

    // Convert the result to amp::Path and return
    amp::Path path;
    if (result.success) {
        path.valid = true;
        for (const auto& node : result.node_path) {
            path.waypoints.push_back(nodes[node]);
        }
    } else {
        path.valid = false;
        std::cerr << "Path not found" << std::endl;
    }

    return path;
}


void MyPRM2D::sampleConfigurations(int n, const amp::GridCSpace2D& collision_checker) {
    // Seed the rng
    amp::RNG::seed(std::chrono::high_resolution_clock::now().time_since_epoch().count());
    tools tools;
    
    int iterationsMax = 10000;
    // Generate n random samples
    for (int i = 0; i < n; ++i) {
        // 2d vector to store sample
        Eigen::VectorXd sample(2);

        // Flagger to check if valid sample is found
        bool valid_sample = false;

        int iterations = 0;
        while (!valid_sample && iterations < iterationsMax) {

            // Generate a random sample
            float x = amp::RNG::srandd(xmin, xmax);
            float y = amp::RNG::srandd(ymin, ymax);
            Eigen::Vector2d checkPoint(x, y);
            // Check if the sample is valid
            bool insideAnyObstacle = false;
            for (const amp::Obstacle2D& obstacle : obstacles) {
                const std::vector<Eigen::Vector2d> polygon = obstacle.verticesCCW();

                if (tools.isPointInsidePolygon(checkPoint, polygon)) {
                    insideAnyObstacle = true;
                    break; // Exit the loop early since we found a collision
                }
            }

            if (!insideAnyObstacle) {
                valid_sample = true;
                sample = checkPoint;
            }

            iterations++;
        }

        if (valid_sample) {
            // Define index/id to identify node based on current size of 'nodes' map
            amp::Node node_id = nodes.size();
            // Add sample to 'nodes' map at index defined  
            nodes[node_id] = sample;
        } else {
            std::cerr << "Failed to find a valid sample after " << iterationsMax << " iterations." << std::endl;
        }
    }
}



bool MyPRM2D::isPathValid(const Eigen::Vector2d& start, const Eigen::Vector2d& end, const amp::GridCSpace2D& collision_checker) {

    auto [x0_cells, x1_cells] = collision_checker.size();
    // Convert start and end points to grid coordinates
    int x1 = (start.x() - xmin) / (xmax - xmin) * (x0_cells - 1);
    int y1 = (start.y() - ymin) / (ymax - ymin) * (x1_cells - 1);
    int x2 = (end.x() - xmin) / (xmax - xmin) * (x0_cells - 1);
    int y2 = (end.y() - ymin) / (ymax - ymin) * (x1_cells - 1);

    // Bresenham's line algorithm
    int dx = abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
    int dy = -abs(y2 - y1), sy = y1 < y2 ? 1 : -1; 
    int err = dx + dy, e2;
    
    while (true) {
        // Check if current grid cell is an obstacle
        if (collision_checker(x1, y1) == 1) {
            return false;
        }
        if (x1 == x2 && y1 == y2) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x1 += sx; }
        if (e2 <= dx) { err += dx; y1 += sy; }
    }
    
    return true;
}
