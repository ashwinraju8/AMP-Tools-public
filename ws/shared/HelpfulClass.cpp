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


bool tools::isIntersecting(const Eigen::Vector2d& p1, const Eigen::Vector2d& q1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q2) const{
    // Calculate the four orientations needed for the general and
    // special cases of orientation
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, q1 and p2 are collinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;

    // p1, q1 and q2 are collinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;

    // p2, q2 and p1 are collinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;

    // p2, q2 and q1 are collinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false; // Doesn't fall in any of the above cases
}

// Utility function to check if point q lies on line segment 'pr'
bool tools::onSegment(const Eigen::Vector2d& p, const Eigen::Vector2d& q, const Eigen::Vector2d& r) const {
    if (q[0] <= std::max(p[0], r[0]) && q[0] >= std::min(p[0], r[0]) &&
        q[1] <= std::max(p[1], r[1]) && q[1] >= std::min(p[1], r[1]))
        return true;
    return false;
}

// To find orientation of ordered triplet (p, q, r).
// The function returns 0 if p, q, and r are collinear,
// 1 if Clockwise, and 2 if Counterclockwise
int tools::orientation(const Eigen::Vector2d& p, const Eigen::Vector2d& q, const Eigen::Vector2d& r) const {
    double val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1]);
    if (val == 0) return 0;  // collinear
    return (val > 0) ? 1 : 2; // clock or counterclockwise
}

Eigen::Vector2d tools::distanceObstacle(const Eigen::Vector2d& q, const amp::Obstacle2D& obstacle) {
    // vector pointing from closest point to robot
    Eigen::Vector2d closestPointVector = Eigen::Vector2d::Zero();
    double minMagnitude = std::numeric_limits<double>::max();
    const std::vector<Eigen::Vector2d> vertices = obstacle.verticesCCW();

    for (size_t i = 0; i < vertices.size(); ++i) {
        const Eigen::Vector2d& a = vertices[i];
        const Eigen::Vector2d& b = vertices[(i + 1) % vertices.size()]; // Next vertex with wrap-around

        Eigen::Vector2d currentClosest = closestPointOnSegment(q, a, b);
        Eigen::Vector2d currentVector = q - currentClosest;
        double currentMagnitude = currentVector.norm();

        if (currentMagnitude < minMagnitude) {
            minMagnitude = currentMagnitude;
            closestPointVector = currentVector;
        }
    }
    return closestPointVector;
}

Eigen::Vector2d tools::closestPointOnSegment(const Eigen::Vector2d& q, const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    Eigen::Vector2d ab = b - a;
    double t = (q - a).dot(ab) / ab.squaredNorm();

    // Handle zero-length segment
    if (ab.squaredNorm() < std::numeric_limits<double>::epsilon()) {
        return a;  // a and b are the same in this case
    }

    if (t < 0.0) return a;
    if (t > 1.0) return b;
    return a + t * ab;
}
