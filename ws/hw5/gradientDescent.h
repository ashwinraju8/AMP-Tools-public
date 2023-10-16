

#pragma once
#include "AMPCore.h"
#include "hw/HW5.h"

namespace amp {
    class MyGradientDescent : public GDAlgorithm {
    public:
        // Constructor that initializes dStar and QStar
        MyGradientDescent(double dstarValue, double QstarValue, double zetaValue, double etaValue);

        Path2D plan(const Problem2D& problem) override;
        Eigen::Vector2d computeGradient(const Eigen::Vector2d& position, const Eigen::Vector2d& goal, const std::vector<Obstacle2D>& obstacles);
        Eigen::Vector2d uAttractiveGradient(const Eigen::Vector2d& q, const Eigen::Vector2d& goal);
        Eigen::Vector2d uRepulsiveGradient(const Eigen::Vector2d& q, const std::vector<Obstacle2D>& obstacles);
        Eigen::Vector2d distanceObstacle(const Eigen::Vector2d& q, const Obstacle2D& obstacle);
        Eigen::Vector2d closestPointOnSegment(const Eigen::Vector2d& q, const Eigen::Vector2d& a, const Eigen::Vector2d& b);

    private:
        double dstar;   // Distance threshold to the goal for convergence
        double Qstar;   // Distance threshold for obstacle influence
        double zeta;
        double eta;
    };
}