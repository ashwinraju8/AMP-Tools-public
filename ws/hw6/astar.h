#include "AMPCore.h"
#include "hw/HW6.h"
#include "hw/HW2.h"
#include <queue>

class MyAStarAlgo : public amp::AStar {
    public:
        virtual amp::AStar::GraphSearchResult search(const amp::ShortestPathProblem& problem, const amp::SearchHeuristic& heuristic) override;
};