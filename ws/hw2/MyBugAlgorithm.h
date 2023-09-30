#pragma once

#include "AMPCore.h"
#include "hw/HW2.h"

/// @brief Declare your bug algorithm class here. Note this class derives the bug algorithm class declared in HW2.h
class MyBugAlgorithm : public amp::BugAlgorithm {
    public:
        // Override and implement the bug algorithm in the plan method. The methods are declared here in the `.h` file
        virtual amp::Path2D plan(const amp::Problem2D& problem) override;
        
        // Add any other methods here... 
        void calculateBarycentricCoordinates(const Eigen::Vector2d& point, const Eigen::Vector2d& p0, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, 
                                    double& t, double& u);

        bool isPointInsidePolygon(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon);

        Eigen::Vector2d moveForward(std::vector<amp::Obstacle2D> obstacles, const Eigen::Vector2d& pointCurrent, const Eigen::Vector2d& direction, 
                                    double stepSize, double turnAngle);

        bool checkRight(std::vector<amp::Obstacle2D> obstacles, const Eigen::Vector2d& pointNext);

        amp::Path2D pathPlanner(const amp::Problem2D& problem, int algorithm);
    
    private:
        // Add any member variables here...
        std::string mode; // define mode across class
        Eigen::Vector2d pointHit; // save point hit for current boundary following mode
};
