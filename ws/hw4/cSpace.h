#pragma once
#include "AMPCore.h"
#include "HelpfulClass.h"
#include "DerivedLinkManipulator.h"

class CSpace2DLinkManipulator : public amp::GridCSpace2D {
public:
    CSpace2DLinkManipulator(
        std::size_t x0_cells, 
        std::size_t x1_cells, 
        double x0_min, double x0_max, 
        double x1_min, double x1_max,
        const amp::LinkManipulator& manipulator,
        const std::vector<amp::Obstacle2D>& obstacles);

    bool inCollision(double x0, double x1) const override;

private:
    const amp::LinkManipulator& m_manipulator;
    std::vector<std::vector<amp::Obstacle2D>> m_minkowskiObstacles;
    tools m_tools; // Instantiate tools class for utility functions

    // Represents each link as a simple polygon 
    std::vector<Eigen::Vector2d> linkAsPolygon(std::size_t linkIndex) const {
        return {m_manipulator.getJointLocation({0, 0}, linkIndex)};
    }

    std::vector<amp::Obstacle2D> m_obstacles; // Store the obstacles
};