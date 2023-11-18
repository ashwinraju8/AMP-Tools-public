#pragma once
#include "AMPCore.h"
#include <Eigen/Core>
#include <vector>
#include <cmath>

class tools {
public:
    bool isPointInsidePolygon(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon) const;
    amp::Obstacle2D computeMinkowskiSum(const std::vector<Eigen::Vector2d>& polygonA, const std::vector<Eigen::Vector2d>& polygonB) const;
    bool isIntersecting(const Eigen::Vector2d& p1, const Eigen::Vector2d& q1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q2) const;
    Eigen::Vector2d distanceObstacle(const Eigen::Vector2d& q, const amp::Obstacle2D& obstacle);
private:
    void calculateBarycentricCoordinates(const Eigen::Vector2d& point, const Eigen::Vector2d& p0, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, 
                                         double& t, double& u) const;
    void ensureCounterClockwise(std::vector<Eigen::Vector2d>& polygon) const;
    amp::Obstacle2D computeConvexHull(const std::vector<Eigen::Vector2d>& points) const;

    bool onSegment(const Eigen::Vector2d& p, const Eigen::Vector2d& q, const Eigen::Vector2d& r) const ;
    int orientation(const Eigen::Vector2d& p, const Eigen::Vector2d& q, const Eigen::Vector2d& r) const ;
    Eigen::Vector2d closestPointOnSegment(const Eigen::Vector2d& q, const Eigen::Vector2d& a, const Eigen::Vector2d& b);
};
