#include "astar.h"

amp::AStar::GraphSearchResult MyAStarAlgo::search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) {

    // problem.graph->print();
    // LOG("init node: " << problem.init_node);
    // LOG("goal node: " << problem.goal_node);

    // Define comparator function for priority queue to order elements by
    auto comparator = [](const std::pair<amp::Node, double>& left, const std::pair<amp::Node, double>& right) { 
        return left.second > right.second; 
    };

    // Define the open list as a priority queue of nodes, sorted by their F cost: pair(node, f)
    std::priority_queue<
        std::pair<amp::Node, double>, std::vector<std::pair<amp::Node, double>>, decltype(comparator)
    > open_list(comparator);
    std::set<amp::Node> open_set; // tracks nodes in the open_list

    // Define the closed list as a set of nodes
    std::set<amp::Node> closed_list;

    // Define maps to hold the G cost, F cost, and parent of each node
    std::unordered_map<amp::Node, double> g_values;
    std::unordered_map<amp::Node, double> f_values;
    std::unordered_map<amp::Node, amp::Node> parents;

    // Initialize
    amp::Node start_node = problem.init_node;
    amp::Node goal_node = problem.goal_node;
    g_values[start_node] = 0;
    f_values[start_node] = heuristic(start_node);
    open_list.emplace(start_node, f_values[start_node]);
    int idx = 0;
    while (!open_list.empty()) {
        // Get the node with the lowest F cost
        amp::Node current_node = open_list.top().first;

        // Check for outdated entries
        double current_f = open_list.top().second;
        if (current_f > f_values[current_node]) {
            open_list.pop();
            continue;
        }

        open_list.pop();
        open_set.erase(current_node);  // Remove from open_set

        // If current_node is the goal, reconstruct the path
        if (current_node == goal_node) {
            // std::cout << "Goal found" << std::endl;
            // std::vector<amp::Node> path;
            // while (current_node != start_node) {
            //     path.push_back(current_node);  
            //     current_node = parents[current_node];
            // }
            // path.push_back(start_node);  
            // std::reverse(path.begin(), path.end());

            std::list<amp::Node> path;
            while (current_node != start_node) {
                path.push_front(current_node);
                current_node = parents[current_node];
            }
            path.push_front(start_node);

            amp::AStar::GraphSearchResult result;
            result.success = true;
            result.node_path = std::move(path);
            result.path_cost = g_values[goal_node];
            // std::cout << "path returned" << std::endl;
            // LOG("af");
            std::cout << "idx: " << idx << std::endl;
            return result;
        }

        // Move the current node to the closed list
        closed_list.insert(current_node);

        // Get the neighbors of the current node
        auto neighbors = problem.graph->children(current_node); 
        auto edges = problem.graph->outgoingEdges(current_node);
        for (std::size_t i = 0; i < neighbors.size(); ++i) {
            amp::Node neighbor = neighbors[i];
            if (closed_list.find(neighbor) != closed_list.end()) {
                continue; // Skip nodes already processed
            }
            // std::cout << "edge weight: " << edges[i] << std::endl;
            double g_tmp = g_values[current_node] + edges[i];
            double f_tmp = g_tmp + heuristic(neighbor);

            // std::cout << "Neighbor: " << neighbor << ", g_tmp: " << g_tmp << ", f_tmp: " << f_tmp << std::endl;
            // if (!f_values.count(neighbor) || f_tmp < f_values[neighbor]) {
            //     std::cout << "Updating node " << neighbor << std::endl;
            //     parents[neighbor] = current_node;
            //     g_values[neighbor] = g_tmp;
            //     f_values[neighbor] = f_tmp;

            //     if (!open_set.count(neighbor)) { // if neighbor is not in open_set, add it to both open_list and open_set
            //         open_list.emplace(neighbor, f_tmp);
            //         open_set.insert(neighbor);
            //     }
            // }
            if (!open_set.count(neighbor) || f_tmp < f_values[neighbor]) {
                parents[neighbor] = current_node;
                g_values[neighbor] = g_tmp;
                f_values[neighbor] = f_tmp;

                open_list.emplace(neighbor, f_tmp);
                open_set.insert(neighbor);
            }
        }

        // LOG("update_info: " << update_info.node << " " << update_info.parent << " " << update_info.g << " " << update_info.h << " " << update_info.f);
        idx += 1;
    }
    // LOG("af");

    // If the goal was not reached, return an empty path.
    std::cout << "No valid path" << std::endl;
    return amp::AStar::GraphSearchResult();
}