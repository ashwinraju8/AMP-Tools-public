#include "AMPCore.h"
#include "hw/HW7.h"
#include "astar.h"
#include "Wavefront.h"
#include "HelpfulClass.h"
#include <chrono>

class MyPRM2D : public amp::PRM2D {
    public:
        virtual amp::Path2D plan(const amp::Problem2D& problem) override {

            auto start = std::chrono::high_resolution_clock::now();

            // Make a collsion checker object
            MyPointWFAlgo wfPoint;
            std::unique_ptr<amp::GridCSpace2D> cSpace = wfPoint.constructDiscretizedWorkspace(problem);

            // Set the grid parameters
            xmin = problem.x_min;
            xmax = problem.x_max;
            ymin = problem.y_min;
            ymax = problem.y_max;
            obstacles = problem.obstacles;

            // int n = 200;  // Number of samples
            // double r = 1;  // Radius for connecting nodes

            // Call the planner
            amp::Path path_nd = plan(problem.q_init, problem.q_goal, n, r, *cSpace);

            // Convert the ND path to a 2D path and return it
            amp::Path2D path_2d;
            path_2d.valid = path_nd.valid; // Copy the validity flag

            if (path_2d.valid) {
                for (const auto& waypoint : path_nd.waypoints) {
                    if (waypoint.size() >= 2) {
                        
                        Eigen::Vector2d point_2d(waypoint[0], waypoint[1]);
                        path_2d.waypoints.push_back(point_2d);
                    } else {
                        
                        std::cerr << "Error: The path waypoints are not 2D." << std::endl;
                        path_2d.valid = false;
                        break;
                    }
                }
            }

            auto end = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
            return path_2d;;
        }

        void visualizePRM(const amp::Problem2D& problem) {

            if (graph.nodes().empty()) {
                std::cerr << "The graph is empty. Visualization cannot be performed." << std::endl;
                return;
            }

            auto node_to_coordinate_2d = [&](amp::Node node) -> Eigen::Vector2d {
                const Eigen::VectorXd& coord_xd = nodes.at(node);
                return Eigen::Vector2d(coord_xd[0], coord_xd[1]);
            };
            amp::Visualizer::makeFigure(problem, graph, node_to_coordinate_2d);

        }

        void prmSettings(int n_, double r_) {
            this->n = n_;
            this->r = r_;
        }

        // Duration in seconds
        double duration = 0.0;  

    private:
        amp::Path plan(const Eigen::VectorXd& init_state,
                       const Eigen::VectorXd& goal_state,
                       int n,
                       int r,
                       const amp::GridCSpace2D& collision_checker);
        void sampleConfigurations(int n, const amp::GridCSpace2D& collision_checker);
        bool isPathValid(const Eigen::Vector2d& start, const Eigen::Vector2d& end, const amp::GridCSpace2D& collision_checker);
        double xmin, xmax, ymin, ymax;
        std::vector<amp::Obstacle2D> obstacles;
        int n = 500; // Number of samples
        double r = 2; // Radius for connecting nodes

        std::unordered_map<amp::Node, Eigen::VectorXd> nodes;
        amp::Graph<double> graph;

};




