#include "cSpace.h"

MyGridCSpace::MyGridCSpace(
    std::size_t x0_cells, 
    std::size_t x1_cells, 
    double x0_min, 
    double x0_max, 
    double x1_min, 
    double x1_max,
    const amp::LinkManipulator& manipulator, 
    const std::vector<amp::Obstacle2D>& obstacles)
: GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max),
  m_manipulator(manipulator),
  m_obstacles(obstacles),
  m_x0_min(x0_min),
  m_x0_max(x0_max),
  m_x1_min(x1_min),
  m_x1_max(x1_max),
  m_x0_cells(x0_cells),
  m_x1_cells(x1_cells)
{
    cSpaceConstructor();
}
void MyGridCSpace::cSpaceConstructor(){
    
    // std::cout<< m_manipulator.nLinks() << std::endl;
    for (std::size_t i = 0; i < m_x0_cells; ++i) {
        for (std::size_t j = 0; j < m_x1_cells; ++j) {
            double theta1 = i * (2 * M_PI)/ m_x0_cells;
            double theta2 = j * (2 * M_PI)/ m_x1_cells;
            // std::cout << theta1 << ", " << theta2 << std::endl;
            
            
            // Create a state from the joint angles
            amp::ManipulatorState state(2); // Create a vector of size 2
            state << theta1, theta2; // Assign the values to the vector
            for (std::size_t k = 0; k < m_manipulator.nLinks(); ++k) {
                // Get the positions of the current joint and the next joint
                Eigen::Vector2d jointStart = m_manipulator.getJointLocation(state, k);
                Eigen::Vector2d jointEnd = m_manipulator.getJointLocation(state, k + 1);
                // std::cout<< "Link: " << k+1 << ", joint start, joint end: (" << jointStart(0) << ", " << jointStart(1) << "), (" << jointEnd(0) << ", " << jointEnd(1) << ")" << std::endl;
                

                // Check each obstacle for intersection with the line segment from jointStart to jointEnd
                for (const auto& obstacle : m_obstacles) {
                    for (std::size_t m = 0; m < obstacle.verticesCCW().size(); ++m) {
                        Eigen::Vector2d vertexStart = obstacle.verticesCCW()[m];
                        Eigen::Vector2d vertexEnd = obstacle.verticesCCW()[(m + 1) % obstacle.verticesCCW().size()];

                        if (m_tools.isIntersecting(jointStart, jointEnd, vertexStart, vertexEnd)) {
                            // std::cout<< "found collision: (" << i << ", " << j << ")" << std::endl;
                            // Collision detected
                            if (i >= 0 && i < m_x0_cells && j >= 0 && j < m_x1_cells) {
                                operator()(i, j) = true;

                                // add buffer to obstacle cells if available
                                if (i+1 < m_x0_cells) operator()(i+1, j) = true;
                                if (j+1 < m_x1_cells) operator()(i, j+1) = true;
                                if (i > 0) operator()(i-1, j) = true; 
                                if (j > 0) operator()(i, j-1) = true; 
                            }
                        }
                    }
                }
            }
        }
    }
}

std::pair<std::size_t, std::size_t> MyGridCSpace::getCellFromPoint(double x0, double x1) const {
    double cellSizeX0 = (m_x0_max - m_x0_min) / m_x0_cells;
    double cellSizeX1 = (m_x1_max - m_x1_min) / m_x1_cells;

    std::size_t indexX0 = static_cast<std::size_t>((x0 - m_x0_min) / cellSizeX0);
    std::size_t indexX1 = static_cast<std::size_t>((x1 - m_x1_min) / cellSizeX1);

    return {indexX0, indexX1};
}

Eigen::Vector2d MyGridCSpace::getPointFromCell(std::size_t indexX0, std::size_t indexX1) const {
    double cellSizeX0 = (m_x0_max - m_x0_min) / m_x0_cells;
    double cellSizeX1 = (m_x1_max - m_x1_min) / m_x1_cells;

    // Calculate the position and add half of the cell size to get the center.
    double x0 = m_x0_min + cellSizeX0 * indexX0 + cellSizeX0 / 2.0; 
    double x1 = m_x1_min + cellSizeX1 * indexX1 + cellSizeX1 / 2.0;

    return Eigen::Vector2d(x0, x1);
}