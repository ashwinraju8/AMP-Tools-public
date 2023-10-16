#pragma once

#include "AMPCore.h"
#include <Eigen/Core>
#include <vector>

class tools {
public:
    bool isPointInsidePolygon(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon) const;
    amp::Obstacle2D computeMinkowskiSum(const std::vector<Eigen::Vector2d>& polygonA, const std::vector<Eigen::Vector2d>& polygonB) const;

private:
    void calculateBarycentricCoordinates(const Eigen::Vector2d& point, const Eigen::Vector2d& p0, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, 
                                         double& t, double& u) const;
    void ensureCounterClockwise(std::vector<Eigen::Vector2d>& polygon) const;
    amp::Obstacle2D computeConvexHull(const std::vector<Eigen::Vector2d>& points) const;
};
