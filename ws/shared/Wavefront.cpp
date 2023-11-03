#include "AMPCore.h"
#include "hw/HW6.h"
#include "Wavefront.h"
#include <queue>


std::unique_ptr<amp::GridCSpace2D> MyPointWFAlgo::constructDiscretizedWorkspace(const amp::Environment2D& environment){

    tools tools;
    double x0_min = environment.x_min;
    double x0_max = environment.x_max;
    double x1_min = environment.y_min;
    double x1_max = environment.y_max;
    std::vector<amp::Obstacle2D> obstacles = environment.obstacles;
    // Define a resolution for the grid
    double resolution = 0.25;  

    // Calculate the number of cells in each dimension
    std::size_t x0_cells = static_cast<std::size_t>((x0_max - x0_min) / resolution);
    std::size_t x1_cells = static_cast<std::size_t>((x1_max - x1_min) / resolution);
    amp::LinkManipulator dummyManipulator({1, 1});
    std::unique_ptr<MyGridCSpace> cSpace = std::make_unique<MyGridCSpace>(x0_cells, 
                                                                        x1_cells, 
                                                                        x0_min, 
                                                                        x0_max, 
                                                                        x1_min, 
                                                                        x1_max, 
                                                                        dummyManipulator, 
                                                                        obstacles);

    double cell_width = resolution;
    double cell_height = resolution;

    // Iterate through each cell in the grid. The variables i and j serve as indices for the cells in the grid.
    for (std::size_t i = 0; i < x0_cells; ++i) {
        for (std::size_t j = 0; j < x1_cells; ++j) {
            // Calculate the cell's boundaries.
            double cell_x_min = x0_min + i * cell_width;
            double cell_x1_min = x1_min + j * cell_height;
            double cell_x_max = cell_x_min + cell_width;
            double cell_x1_max = cell_x1_min + cell_height;

            // Define the corners of the current cell.
            Eigen::Vector2d bottom_left(cell_x_min, cell_x1_min);
            Eigen::Vector2d bottom_right(cell_x_max, cell_x1_min);
            Eigen::Vector2d top_left(cell_x_min, cell_x1_max);
            Eigen::Vector2d top_right(cell_x_max, cell_x1_max);

            // Construct line segments for each of the four sides of the cell.
            std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> cell_edges = {
                {bottom_left, bottom_right},
                {bottom_right, top_right},
                {top_right, top_left},
                {top_left, bottom_left}
            };

            // Check if any of the cell's edges intersect with any of the obstacles' edges.
            for (const auto& obstacle : obstacles) {
                // Iterate through each edge of the obstacle.
                for (std::size_t m = 0; m < obstacle.verticesCCW().size(); ++m) {
                    // Define the start and end points of the current obstacle edge.
                    Eigen::Vector2d vertexStart = obstacle.verticesCCW()[m];
                    Eigen::Vector2d vertexEnd = obstacle.verticesCCW()[(m + 1) % obstacle.verticesCCW().size()]; // Loop back to the first vertex for the last edge

                    // Check for intersection between the current obstacle edge and each cell edge.
                    for (const auto& cell_edge : cell_edges) {
                        // If they intersect, mark the cell as occupied and move to the next cell.
                        if (tools.isIntersecting(cell_edge.first, cell_edge.second, vertexStart, vertexEnd)) {
                            // Mark the current cell and immediate neighbors as occupied or non-traversable.
                            (*cSpace)(i,j) = true;
                            // (*cSpace)(i+1,j) = true;
                            // (*cSpace)(i,j+1) = true;
                            // (*cSpace)(i-1,j) = true;
                            // (*cSpace)(i,j-1) = true;
                            // Jump to the next cell, skipping further checks for the current cell.
                            goto next_cell;
                        }
                        else{
                            (*cSpace)(i,j) = false;
                        }
                    }
                }
            }
            // Label used as the target for 'goto'. When 'goto next_cell;' is executed, the program jumps here.
            next_cell: continue;
        }
    }
            
    return cSpace;
}
        

amp::Path2D MyPointWFAlgo::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace){

    // Possible moves
    const std::vector<std::pair<int, int>> moves = {
        {0, 1},  // move right
        {1, 0},  // move down
        {0, -1}, // move left
        {-1, 0}  // move up
    };

    auto [x0_cells, x1_cells] = grid_cspace.size();
    amp::DenseArray2D<int> wavefront(x0_cells, x1_cells, 0);

    // represent wavefront grid as a 2D vector of integers.
    // -1 means obstacle.
    for (std::size_t i = 0; i < x0_cells; ++i) {
        for (std::size_t j = 0; j < x1_cells; ++j) {
            if(grid_cspace(i,j) == 1){
                wavefront(i,j) = -1;
            }
        }
     }

    // Convert start and goal points to grid cells
    auto start_cell = grid_cspace.getCellFromPoint(q_init[0], q_init[1]);
    auto goal_cell = grid_cspace.getCellFromPoint(q_goal[0], q_goal[1]);

    std::cout << "start cell: (" << start_cell.first << ", " << start_cell.second << ")" << std::endl;
    std::cout << "goal cell: (" << goal_cell.first << ", " << goal_cell.second << ")" << std::endl;

    // std::cout << "start cell value: " << wavefront(start_cell.first, start_cell.second) << std::endl;
    // std::cout << "goal cell value: " << wavefront(goal_cell.first,goal_cell.second) << std::endl;


    // Create a queue and add the starting point to it.
    std::queue<std::pair<std::size_t, std::size_t>> q;
    q.push(start_cell);

    while (!q.empty()) {
        // std::cout << "in while loop" << std::endl;
        auto current = q.front();
        q.pop();

        // If this is the goal point.
        if (current == goal_cell) {
            std::cout << "Goal found at: (" << current.first << ", " << current.second << ")" << std::endl;
            
            // Construct the path from the goal to the start by following the wavefront back.
            amp::Path2D path;
            path.waypoints.push_back(q_goal);
            while (current != start_cell) {
                // std::cout << "Current: (" << current.first << ", " << current.second << "), Value: " << wavefront(current.first, current.second) << std::endl;
                path.waypoints.push_back(grid_cspace.getPointFromCell(current.first, current.second));
                int min_val = std::numeric_limits<int>::max();
                std::pair<std::size_t, std::size_t> next_step = {0, 0}; // Initialize to an invalid value

                // Mark the current cell as visited
                wavefront(current.first, current.second) = -2; 

                for (auto& move : moves) {
                    std::pair<int, int> neighbor(current.first + move.first, current.second + move.second);

                    // Check if the neighbor is the start cell
                    if (std::pair<int, int>(neighbor.first, neighbor.second) == std::pair<int, int>(start_cell.first, start_cell.second)) {
                        next_step = start_cell;
                        break;
                    }
                    
                    if (neighbor.first >= 0 && neighbor.first < x0_cells &&
                        neighbor.second >= 0 && neighbor.second < x1_cells) {

                        int neighbor_val = wavefront(neighbor.first, neighbor.second);
                        // std::cout << "  Neighbor: (" << neighbor.first << ", " << neighbor.second << "), Value: " << neighbor_val << std::endl;
                        
                        if (neighbor_val != -2 && neighbor_val > 0 && neighbor_val < min_val) { 
                            min_val = neighbor_val;
                            next_step = neighbor;

                        } else if (neighbor_val == min_val) {
                            // check the neighbors of the current neighbor to decide best next step
                            int min_neighbor_val = std::numeric_limits<int>::max();
                            for (auto& sub_move : moves) {
                                std::pair<int, int> sub_neighbor(neighbor.first + sub_move.first, neighbor.second + sub_move.second);
                                if (sub_neighbor.first >= 0 && sub_neighbor.first < x0_cells &&
                                    sub_neighbor.second >= 0 && sub_neighbor.second < x1_cells) 
                                {
                                    int sub_neighbor_val = wavefront(sub_neighbor.first, sub_neighbor.second);
                                    if (sub_neighbor_val != -2 && sub_neighbor_val > 0) 
                                    { 
                                        min_neighbor_val = std::min(min_neighbor_val, sub_neighbor_val);
                                    }
                                }
                            }

                            // If this neighbor of the neighbor has a smaller minimum neighbor value
                            if (min_neighbor_val < min_val) {
                                next_step = neighbor;

                            }
                        } 
                    }
                }
                // std::cout << "Next Step: (" << next_step.first << ", " << next_step.second << ")" << std::endl;
                if (next_step.first == 0 && next_step.second == 0) {
                    std::cout << "No valid next step found. Breaking out of the loop." << std::endl;
                    break; // Break out of the loop if no valid next step was found
                }
                current = next_step;
                
            }

            path.waypoints.push_back(q_init);
            std::reverse(path.waypoints.begin(), path.waypoints.end());
            return path;
        }  

        for (auto move : moves) {
            std::pair<int, int> neighbor(current.first + move.first, current.second + move.second);

            // if (neighbor.first < 0) neighbor.first = x0_cells - 1;
            // if (neighbor.second < 0) neighbor.second = x1_cells - 1;

            // if (neighbor.first >= x0_cells) neighbor.first = 0;
            // if (neighbor.second >= x0_cells) neighbor.second = 0;


            // Check boundaries and if the cell is traversable.
            if (neighbor.first >= 0 && neighbor.first < x0_cells && 
                neighbor.second >= 0 && neighbor.second < x1_cells &&
                wavefront(neighbor.first, neighbor.second) == 0 && 
                wavefront(neighbor.first, neighbor.second) != -1) 
            {
                wavefront(neighbor.first, neighbor.second) = wavefront(current.first, current.second) + 1;
                // std::cout<< wavefront(neighbor.first, neighbor.second) << std::endl;
                q.push(neighbor);
            }
           
            // Check boundaries and if the cell is traversable.
            // if (wavefront(neighbor.first, neighbor.second) == 0 && 
            //     wavefront(neighbor.first, neighbor.second) != -1) 
            // {
            //     wavefront(neighbor.first, neighbor.second) = wavefront(current.first, current.second) + 1;
            //     // std::cout<< wavefront(neighbor.first, neighbor.second) << std::endl;
            //     q.push(neighbor);
            // }
        }
    }

    // If the goal was not reached, return an empty path.
    std::cout << "No valid path" << std::endl;
    return amp::Path2D();

    
}


amp::Path2D MyManipWFAlgo::planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace){
    
    // amp::ManipulatorState state_init = manipulator.getConfigurationFromIK(q_init);
    // amp::ManipulatorState state_goal = manipulator.getConfigurationFromIK(q_goal);

    // std::cout << manipulator.getJointLocation(state_init, 2) << std::endl;
    // Eigen::Vector2d state_init2d;
    // state_init2d << state_init(0), state_init(1);

    // Eigen::Vector2d state_goal2d;
    // state_goal2d << state_goal(0), state_goal(1);

    // std::cout << "start config: (" << state_init2d(0) << ", " << state_init2d(1) << ")" << std::endl;
    // std::cout << "goal config: (" << state_goal2d(0) << ", " << state_goal2d(1) << ")" << std::endl;

    MyPointWFAlgo myPointWFAlgo;
    // amp::Path2D anglePath = myPointWFAlgo.planInCSpace(state_init2d, state_goal2d, grid_cspace);
    amp::Path2D anglePath = myPointWFAlgo.planInCSpace(q_init, q_goal, grid_cspace);
    return anglePath;

}