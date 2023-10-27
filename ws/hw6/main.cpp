#include "AMPCore.h"
#include "hw/HW6.h"
#include "hw/HW2.h"
#include "Wavefront.h"
#include "astar.h"

using namespace amp;

void exercise1(){

    MyPointWFAlgo wfPoint;
    // Use WO1 from Exercise 2
    // amp::Problem2D problem = amp::HW2::getWorkspace1();
    amp::Problem2D problem = amp::HW2::getWorkspace2();
    std::unique_ptr<amp::GridCSpace2D> cSpace = wfPoint.constructDiscretizedWorkspace(problem);
    amp::Path2D path = wfPoint.planInCSpace(problem.q_init, problem.q_goal, *cSpace);
    std::cout << "Path Length: " << path.length() << std::endl;
    amp::HW6:: checkPointAgentPlan(path, problem);

    Visualizer::makeFigure(*cSpace);
    Visualizer::showFigures();

    Visualizer::makeFigure(problem, path);
    Visualizer::showFigures();
    
}

void exercise2(){

    MyPointWFAlgo wfLink;
    // Use WO1 from Exercise 2
    // amp::Problem2D problem = amp::HW6::getHW4Problem1();
    // amp::Problem2D problem = amp::HW6::getHW4Problem2();
    amp::Problem2D problem = amp::HW6::getHW4Problem3();

    Eigen::Vector2d q_init;
    q_init << -2, 0;

    Eigen::Vector2d q_goal;
    q_goal << 2, 0;

    double x0_min = 0;
    double x0_max = 2*M_PI;
    double x1_min = 0;
    double x1_max = 2*M_PI;

    std::size_t x0_cells = 50;
    std::size_t x1_cells = 50;

    std::vector<double> linkLengths = {1, 1};
    LinkManipulator manipulator(linkLengths);

    std::shared_ptr<MyGridCSpace2DConstructor> gridCSpace = std::make_shared<MyGridCSpace2DConstructor>(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max);
    std::unique_ptr<amp::GridCSpace2D> cSpace = gridCSpace->construct(manipulator, problem);

    MyManipWFAlgo algo(manipulator, gridCSpace);

    amp::ManipulatorState state_init = manipulator.getConfigurationFromIK(q_init);
    amp::ManipulatorState state_goal = manipulator.getConfigurationFromIK(q_goal);

    std::cout << manipulator.getJointLocation(state_init, 2) << std::endl;
    Eigen::Vector2d state_init2d;
    state_init2d << state_init(0), state_init(1);

    Eigen::Vector2d state_goal2d;
    state_goal2d << state_goal(0), state_goal(1);


    amp::Path2D path = algo.planInCSpace(state_init2d, state_goal2d, *cSpace);
    std::cout << "Path Length: " << path.length() << std::endl;
    amp::HW6:: checkLinkManipulatorPlan(path, manipulator, problem);

    Visualizer::makeFigure(problem, manipulator, path);

    Visualizer::makeFigure(*cSpace, path);
    Visualizer::showFigures();
        
}

void exercise3(){
    amp::ShortestPathProblem problem = HW6::getEx3SPP();
    amp::LookupSearchHeuristic heuristic = HW6::getEx3Heuristic();

    MyAStarAlgo astar;
    amp::AStar::GraphSearchResult path = astar.search(problem, heuristic);
    amp::HW6:: checkGraphSearchResult(path, problem, heuristic);
    std::cout << "Path Cost: " << path.path_cost << std::endl;
    std::cout << "Node path: ";
    for (const auto& element : path.node_path) {
        std::cout << element << " ";
    }
    std::cout << std::endl;


    // for (const auto& node : problem.graph->nodes()) {
    //     std::cout << "Node " << node << " has neighbors: ";
    //     auto neighbors = problem.graph->children(node);
    //     auto edges = problem.graph->outgoingEdges(node);
    //     for (std::size_t i = 0; i < neighbors.size(); ++i) {
    //         std::cout << "(" << neighbors[i] << ", " << edges[i] << ") ";
    //     }
    //     std::cout << std::endl;
    // }
    
}

int main(int argc, char** argv) {
    // amp::RNG::seed(amp::RNG::randiUnbounded());

    // exercise1();
    // exercise2();
    exercise3();
    
    // amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>("nonhuman.biologic@myspace.edu", argc, argv, std::make_tuple(), std::make_tuple("hey therre"), std::make_tuple());
    std::vector<double> linkLengths = {1, 1};
    LinkManipulator manipulator(linkLengths);
    double x0_min = 0;
    double x0_max = 2*M_PI;
    double x1_min = 0;
    double x1_max = 2*M_PI;
    std::size_t x0_cells = 50;
    std::size_t x1_cells = 50;
    std::shared_ptr<MyGridCSpace2DConstructor> gridCSpace = std::make_shared<MyGridCSpace2DConstructor>(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max);
    
    amp::HW6::grade<MyPointWFAlgo, MyManipWFAlgo, MyAStarAlgo>(
        "asra8222@colorado.edu", argc, argv,
        std::make_tuple(), 
        std::make_tuple(manipulator, gridCSpace), 
        std::make_tuple()
    );
    
    return 0;
}


    //  MyAStarAlgo aStarAlgo;
    //  uint32_t n_nodes = 20;
    //  double min_edge_weight = 0.0;
    //  double max_edge_weight = 10.0;
    //  uint32_t max_outgoing_edges_per_node = 5;
    //  double connectedness = 0.5;

    //  amp::ShortestPathProblem randPath;
    //  amp::AStar::GraphSearchResult randResult;
    
    //  for (int i = 0; i < 20; i++) {
    //      uint32_t seed = i;
    //      randPath = amp::GraphTools::generateRandomSPP(n_nodes, min_edge_weight, max_edge_weight, max_outgoing_edges_per_node, connectedness, seed);
    //      randResult = aStarAlgo.search(randPath, amp::SearchHeuristic());
    //      amp::HW6::checkGraphSearchResult(randResult, randPath, amp::SearchHeuristic(), true);
    //     PAUSE;
    // }