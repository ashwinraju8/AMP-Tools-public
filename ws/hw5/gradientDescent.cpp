#include "gradientDescent.h"


namespace amp {
MyGradientDescent::MyGradientDescent(double dStarValue, double QStarValue, double zetaValue, double etaValue) 
    : dstar(dStarValue), Qstar(QStarValue), zeta(zetaValue), eta(etaValue) {}

Path2D MyGradientDescent::plan(const Problem2D& problem) {
    const Eigen::Vector2d q_init = problem.q_init;
    const Eigen::Vector2d q_goal = problem.q_goal;
    Eigen::Vector2d currentPosition = q_init;
    std::vector<Eigen::Vector2d> waypoints = {currentPosition};


    int iteration = 0;
    int maxIterations = 2000; 
    double stepSize = 0.05;  
    double epsilon = 0.25;    

    // Seed the random number generator with the current time
    // std::srand(static_cast<unsigned int>(std::time(nullptr)));
    Eigen::Vector2d perturbation; 
    // std::cout << perturbation << std::endl;
    double perturbationScale = 1; // Adjust this based on your problem's scale

    while (iteration < maxIterations) {
        Eigen::Vector2d gradient = computeGradient(currentPosition, q_goal, problem.obstacles);
        
        perturbation = 0.5*Eigen::Vector2d::Random(); // A random vector with values between -1 and 1
        if ((currentPosition - q_goal).norm() > epsilon && gradient.norm() < epsilon) {
            perturbation = Eigen::Vector2d::Random(); // A random vector with values between -1 and 1
            // std::cout << "perturbation added: " << perturbation << std::endl;
            perturbation = perturbationScale * perturbation;
        }
        gradient += perturbation;

        currentPosition -= stepSize * gradient;
        waypoints.push_back(currentPosition);

        if ((currentPosition - q_goal).norm() < epsilon) {
            std::cout << "Goal Reached!" << std::endl;
            waypoints.push_back(q_goal);
            break;
        }
        
        iteration++;
    }
    if (iteration == maxIterations){
        std::cout << "max iterations reached" << std::endl;
    }

    Path2D path;
    path.waypoints = waypoints; 
    return path;
}

Eigen::Vector2d MyGradientDescent::computeGradient(const Eigen::Vector2d& position, const Eigen::Vector2d& goal, const std::vector<Obstacle2D>& obstacles) {
    Eigen::Vector2d gradientAttraction = uAttractiveGradient(position, goal);
    Eigen::Vector2d gradientRepulsion = uRepulsiveGradient(position, obstacles);
    return gradientAttraction + gradientRepulsion;
}

Eigen::Vector2d MyGradientDescent::uAttractiveGradient(const Eigen::Vector2d& q, const Eigen::Vector2d& goal) {
    Eigen::Vector2d gradientAttraction;
    if ((q-goal).norm() <= dstar){
        gradientAttraction = zeta * (q - goal);
    }
    else{
        gradientAttraction = dstar*zeta*(q-goal)/(q-goal).norm();
    }
    return gradientAttraction;
}

Eigen::Vector2d MyGradientDescent::uRepulsiveGradient(const Eigen::Vector2d& q, const std::vector<Obstacle2D>& obstacles) {
    Eigen::Vector2d totalRepulsiveGradient = Eigen::Vector2d::Zero();

    for (const Obstacle2D& obstacle : obstacles) {
        Eigen::Vector2d distanceVector = distanceObstacle(q, obstacle);
        double distanceMagnitude = distanceVector.norm();
        
        // If the point is inside the obstacle or within some threshold distance, compute the repulsive force.
        if (distanceMagnitude <= Qstar && distanceMagnitude > 0) {
            totalRepulsiveGradient += eta * (1.0 / Qstar - 1.0 / distanceMagnitude) * distanceVector / (distanceMagnitude * distanceMagnitude * distanceMagnitude);
        }
    }

    return totalRepulsiveGradient;
}


Eigen::Vector2d MyGradientDescent::distanceObstacle(const Eigen::Vector2d& q, const Obstacle2D& obstacle) {
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

Eigen::Vector2d MyGradientDescent::closestPointOnSegment(const Eigen::Vector2d& q, const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
    Eigen::Vector2d ab = b - a;
    double t = (q - a).dot(ab) / ab.squaredNorm();
    if (t < 0.0) return a;
    if (t > 1.0) return b;
    return a + t * ab;
}

} // end of namespace amp