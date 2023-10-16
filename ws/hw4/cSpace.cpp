#include "cSpace.h"

CSpace2DLinkManipulator::CSpace2DLinkManipulator(
    std::size_t x0_cells, 
    std::size_t x1_cells, 
    double x0_min, double x0_max, 
    double x1_min, double x1_max,
    const amp::LinkManipulator& manipulator,
    const std::vector<amp::Obstacle2D>& obstacles)
    : GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max),
      m_manipulator(manipulator),
      m_obstacles(obstacles)
{
    // Initialize m_minkowskiObstacles
    m_minkowskiObstacles.resize(m_manipulator.nLinks());

    // Compute the Minkowski sum for each obstacle and each link
    for (const auto& obstacle : m_obstacles) {
        for (std::size_t i = 0; i < m_manipulator.nLinks(); ++i) {
            m_minkowskiObstacles[i].push_back(m_tools.computeMinkowskiSum(linkAsPolygon(i), obstacle.verticesCCW()));
        }
    }

    // Construct the C-space for 2-link manipulator by checking each configuration
    for(std::size_t i = 0; i < x0_cells; ++i) {
        for(std::size_t j = 0; j < x1_cells; ++j) {
            double theta1 = x0_min + i * (x0_max - x0_min) / x0_cells;
            double theta2 = x1_min + j * (x1_max - x1_min) / x1_cells;
            (*this)(i, j) = inCollision(theta1, theta2);
        }
    }
}

bool CSpace2DLinkManipulator::inCollision(double x0, double x1) const {
    // Convert x0 and x1 to joint angles and use the manipulator to get the link positions
    amp::ManipulatorState state = {x0, x1};
    for(std::size_t i = 0; i < m_manipulator.nLinks(); ++i) {
        Eigen::Vector2d joint_pos = m_manipulator.getJointLocation(state, i);
        for(const auto& minkowskiObstacle : m_minkowskiObstacles[i]) {
            const std::vector<Eigen::Vector2d> minkowskiPolygon = minkowskiObstacle.verticesCCW();
            if(m_tools.isPointInsidePolygon(joint_pos, minkowskiPolygon)) {
                return true; // Collision detected
            }
        }
    }
    return false; // No collision detected
}
