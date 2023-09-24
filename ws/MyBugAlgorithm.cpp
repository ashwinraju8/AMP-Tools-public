#include "MyBugAlgorithm.h"

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::plan(const amp::Problem2D& problem) {

    // Your algorithm solves the problem and generates a path. Here is a hard-coded to path for now...
    amp::Path2D path;
    int algorithm = 2;
    path = pathPlanner(problem, algorithm);

    return path;
}

// Define a function to calculate the barycentric coordinates
void MyBugAlgorithm::calculateBarycentricCoordinates(const Eigen::Vector2d& point, const Eigen::Vector2d& p0, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, 
                                    double& t, double& u) 
{
    double detT = (p1.x() - p0.x()) * (p2.y() - p0.y()) - (p2.x() - p0.x()) * (p1.y() - p0.y());
    t = ((point.x() - p0.x()) * (p2.y() - p0.y()) - (p2.x() - p0.x()) * (point.y() - p0.y())) / detT;
    u = ((p1.x() - p0.x()) * (point.y() - p0.y()) - (point.x() - p0.x()) * (p1.y() - p0.y())) / detT;
}

// Define a function to check if a point is inside a polygon
bool MyBugAlgorithm::isPointInsidePolygon(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& polygon) {
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

Eigen::Vector2d MyBugAlgorithm::moveForward(std::vector<amp::Obstacle2D> obstacles, const Eigen::Vector2d& pointCurrent, const Eigen::Vector2d& direction, 
                                            double stepSize, double turnAngle) {
    
    Eigen::Vector2d errorVector(10000,10000);
    Eigen::Vector2d stepVector = direction.normalized() * stepSize;
    Eigen::Vector2d pointNext = pointCurrent + stepVector;
    Eigen::Vector2d pointRight(pointCurrent.x() + stepVector.y(), pointCurrent.y() - stepVector.x());

    double maxTurnAngleThreshold = 2.0 * M_PI;
     
    for (const amp::Obstacle2D& obstacle : obstacles) {
        const std::vector<Eigen::Vector2d> polygon = obstacle.verticesCCW();
        double totalTurnAngle = 0.0; // Track the total turning angle

        if (isPointInsidePolygon(pointNext, polygon)) {
            if (mode == "go2goal") {
                // std::cout<<"leave go2goal \n";
                mode = "boundary following";
                pointHit = pointNext;
            }
            // std::cout<<"collision \n";
            Eigen::Vector2d frontVec;
            Eigen::Vector2d rightVec;
            while (isPointInsidePolygon(pointNext, polygon) || !isPointInsidePolygon(pointRight, polygon)) {
                // Turn left incrementally
            

                frontVec = pointNext - pointCurrent;
                frontVec = frontVec.normalized() * stepSize;
                rightVec = pointRight - pointCurrent;
                rightVec = rightVec.normalized() * stepSize;


                // rotate frontVec
                double xFront = frontVec.x();
                double yFront = frontVec.y();
                double new_xFront = xFront*cos(turnAngle) - yFront*sin(turnAngle);
                double new_yFront = xFront*sin(turnAngle) + yFront*cos(turnAngle);
                frontVec.x() = new_xFront;
                frontVec.y() = new_yFront;
                pointNext = pointCurrent + frontVec;
                // rotate rightVec
                double xRight = rightVec.x();
                double yRight = rightVec.y();
                double new_xRight = xRight*cos(turnAngle) - yRight*sin(turnAngle);
                double new_yRight = xRight*sin(turnAngle) + yRight*cos(turnAngle);
                rightVec.x() = new_xRight;
                rightVec.y() = new_yRight;
                pointRight = pointCurrent + rightVec;
                
                // Update the total turning angle
                totalTurnAngle += turnAngle;
                
                // Break out of the while loop if the maximum turning angle is reached
                if (totalTurnAngle >= maxTurnAngleThreshold) {
                    // std::cout << "turned 360"<< std::endl;
                    return errorVector;
                }
            }
        } 
    }
    
    return pointNext;
}

// Implement your methods in the `.cpp` file, for example:
amp::Path2D MyBugAlgorithm::pathPlanner(const amp::Problem2D& problem, int algorithm) {

    const Eigen::Vector2d q_init = problem.q_init;
    const Eigen::Vector2d q_goal = problem.q_goal;
    std::vector<amp::Obstacle2D> obstacles = problem.obstacles;

    mode = "go2goal"; // or "boundary following" needs to be member variable

    Eigen::Vector2d pointCurrent = q_init;
    Eigen::Vector2d mLine = q_goal - q_init;
    Eigen::Vector2d direction;
    Eigen::Vector2d errorVector(10000,10000);
    Eigen::Vector2d emptyVector;
    amp::Path2D emptyPath;

    amp::Path2D path;
    path.waypoints.push_back(q_init);

    Eigen::Vector2d pointLeaveBug1;
    double dist2goalMin = mLine.norm();
    bool bug1exit;

    double stepSize;
    double turnAngle;
    double error = 0.1;
    double maxTurnAngleThreshold = 2.0 * M_PI;

    int idxCurrent = 0;

    while (idxCurrent<5000) {

        Eigen::Vector2d pointNext;
        if (mode == "go2goal"){

            stepSize = 0.1;
            turnAngle = 0.1;
            pointNext = moveForward(obstacles, pointCurrent, q_goal - pointCurrent, stepSize, turnAngle);

        }
        else if (mode =="boundary following"){
            stepSize = 0.05;
            turnAngle = 0.05;

            Eigen::Vector2d pointPrev = path.waypoints[path.waypoints.size() - 2]; // Get the point before pointCurrent
            direction = pointCurrent - pointPrev;
            pointNext = moveForward(obstacles, pointCurrent, direction, stepSize, turnAngle);

            
            if (std::abs(pointNext.x() - errorVector.x()) < error){
                std::cout << "------------------ERROR: Robot rotated 360 degrees------------------"<< std::endl;
                return path;
            }
            direction = pointNext - pointCurrent;

            // Maintain right point in the obstacle 
            Eigen::Vector2d stepVector = direction.normalized() * stepSize;
            Eigen::Vector2d pointRight(pointCurrent.x() + stepVector.y(), pointCurrent.y() - stepVector.x());

            bool rightIn = 0;
            // Iterate through obstacles
            for (const amp::Obstacle2D& obstacle : obstacles) {
                const std::vector<Eigen::Vector2d> polygon = obstacle.verticesCCW();
                // Check if pointRight is inside the polygon
                if (isPointInsidePolygon(pointRight, polygon)) {
                    rightIn = 1;
                }
                
            }
            if (rightIn == 0){
                // std::cout << "right out\n";
                pointNext = pointRight;
            }

            // Bug2 handling: Check if pointNext is on the line q_goal - q_init
            if (algorithm == 2){
                if ( std::abs((q_init-pointNext).norm() + (q_goal-pointNext).norm() - mLine.norm()) < error) {
                    mode = "go2goal"; // Switch to go2goal mode
                    // std::cout<<"leave boundary following \n";
                }
            }


            // Bug1 handling: Track minimum distance to goal and pointLeaveBug1
            if (algorithm == 1){
                double dist2goal = (q_goal - pointNext).norm();
                if (dist2goal < dist2goalMin) {
                    dist2goalMin = dist2goal;
                    pointLeaveBug1 = pointNext;
                }
            }

            // check if polygon hit point reached
            if ( ((pointNext - pointHit).norm() < error) && ((pointPrev - pointHit).norm() <= error) ) {
                if (algorithm == 1){
                    bug1exit = true;
                }
                else if (algorithm == 2 && mode == "boundary following") {
                    std::cout << "------------------ERROR: Robot traversed entire obstacle (bug2)------------------"<< std::endl;
                    return path;
                }
            }
            if (algorithm == 1 && (pointNext - pointLeaveBug1).norm() < error && bug1exit == true){
                bug1exit = false;
                mode = "go2goal";
                // std::cout<<"leave boundary following \n";
            }

        }
        pointCurrent = pointNext;
        path.waypoints.push_back(pointCurrent);
        idxCurrent++;

        if ((q_goal - pointCurrent).norm() < error){
            pointCurrent = q_goal;
            path.waypoints.push_back(q_goal);
            break;
        }
    }

    return path;
}

