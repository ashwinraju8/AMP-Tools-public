#include "HelpfulClass.h"


void tools::ensureCounterClockwise(std::vector<Eigen::Vector2d>& polygon) const {
    double area = 0.0;
    for (std::size_t i = 0; i < polygon.size(); ++i) {
        const Eigen::Vector2d& p1 = polygon[i];
        const Eigen::Vector2d& p2 = polygon[(i + 1) % polygon.size()];
        area += (p1[0] * p2[1] - p2[0] * p1[1]);
    }

    // If area is negative, the points are in CW order, so reverse them
    if (area < 0.0) {
        std::reverse(polygon.begin(), polygon.end());
    }
}

void tools::calculateBarycentricCoordinates(const Eigen::Vector2d& point, const Eigen::Vector2d& p0, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, 
                                            double& t, double& u) const 
{
    double detT = (p1.x() - p0.x()) * (p2.y() - p0.y()) - (p2.x() - p0.x()) * (p1.y() - p0.y());
    t = ((point.x() - p0.x()) * (p2.y() - p0.y()) - (p2.x() - p0.x()) * (point.y() - p0.y())) / detT;
    u = ((p1.x() - p0.x()) * (point.y() - p0.y()) - (point.x() - p0.x()) * (p1.y() - p0.y())) / detT;
}
 bool tools::isPointInsidePolygon(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon) const {

    // http://wscg.zcu.cz/wscg2004/Papers_2004_Full/B83.pdf
    if (polygon.size() < 3) {
        // Cannot check, return false for empty or insufficient vertices
        return false;
    }

    // Calculate the centroid of the polygon
    Eigen::Vector2d centroid(0.0, 0.0);
    for (const Eigen::Vector2d& vertex : polygon) {
        centroid += vertex;
    }
    centroid /= polygon.size();

    // Triangulate the polygon
    std::vector<std::tuple<Eigen::Vector2d, Eigen::Vector2d, Eigen::Vector2d>> triangles;

    for (size_t i = 0; i < polygon.size(); i++) {
        triangles.emplace_back(centroid, polygon[i], polygon[(i + 1) % polygon.size()]);
    }

    // Check if the point is inside any of the triangles
    for (const auto& triangle : triangles) {
        double t, u;
        calculateBarycentricCoordinates(point, std::get<0>(triangle), std::get<1>(triangle), std::get<2>(triangle), t, u);
        if (t >= 0.0 && u >= 0.0 && (t + u) <= 1.0) {
            return true; // Point is inside a triangle, and thus inside the polygon
        }
    }

    return false; // Point is outside all triangles, and thus outside the polygon
}

amp::Obstacle2D tools::computeMinkowskiSum(const std::vector<Eigen::Vector2d>& polygonA, const std::vector<Eigen::Vector2d>& polygonB) const {  
    std::vector<Eigen::Vector2d> minkowskiSum;

    // Work with a copy to not modify the original polygonB
    std::vector<Eigen::Vector2d> polygonB_CCW = polygonB;
    ensureCounterClockwise(polygonB_CCW);

    for (const auto& vertexA : polygonA) {
        for (const auto& vertexB : polygonB_CCW) {
            minkowskiSum.push_back(vertexA + vertexB);
        }
    }
    // Compute and return the convex hull of the Minkowski sum
    return computeConvexHull(minkowskiSum);
}

amp::Obstacle2D tools::computeConvexHull(const std::vector<Eigen::Vector2d>& points) const {
    std::vector<Eigen::Vector2d> workingPoints = points;
    if (workingPoints.size() < 3) return workingPoints;  // Can't form a polygon with less than 3 points

    std::vector<Eigen::Vector2d> sortedPoints = points;
    
    // Step 1: Find the pivot (lowest y-coordinate and leftmost)
    auto pivotIt = std::min_element(sortedPoints.begin(), sortedPoints.end(), 
        [](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
            return (a[1] < b[1]) || (a[1] == b[1] && a[0] < b[0]);
        });
    Eigen::Vector2d pivot = *pivotIt;
    std::swap(sortedPoints[0], *pivotIt);
    
    // Step 2: Sort based on polar angle with pivot
    std::sort(sortedPoints.begin() + 1, sortedPoints.end(),
        [&pivot](const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
            double orientation = (a[1] - pivot[1]) * (b[0] - pivot[0]) - (b[1] - pivot[1]) * (a[0] - pivot[0]);
            if (orientation == 0) {  // Collinear
                return (a - pivot).norm() < (b - pivot).norm();
            }
            return orientation > 0;  // Check for counter-clockwise orientation
        });
    
    // Steps 3-4: Construct the convex hull
    std::vector<Eigen::Vector2d> hull = {sortedPoints[0], sortedPoints[1], sortedPoints[2]};
    for (int i = 3; i < sortedPoints.size(); ++i) {
        while (hull.size() > 1 && 
               (sortedPoints[i][1] - hull.back()[1]) * (hull[hull.size() - 2][0] - hull.back()[0]) - 
               (hull[hull.size() - 2][1] - hull.back()[1]) * (sortedPoints[i][0] - hull.back()[0]) <= 0) {
            hull.pop_back();
        }
        hull.push_back(sortedPoints[i]);
    }
    
    return amp::Obstacle2D(hull);
}
