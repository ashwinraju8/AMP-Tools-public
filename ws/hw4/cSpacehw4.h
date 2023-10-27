// #pragma once
// #include "AMPCore.h"
// #include "HelpfulClass.h"
// #include "DerivedLinkManipulator.h"
// #include "hw/HW4.h"

// class MyGridCSpace : public amp::GridCSpace2D {
// public:
//     MyGridCSpace(std::size_t x0_cells, std::size_t x1_cells, 
//                             double x0_min, double x0_max, 
//                             double x1_min, double x1_max,
//                             const amp::LinkManipulator& manipulator, 
//                             const std::vector<amp::Obstacle2D>& obstacles);
//     void cSpaceConstructor();
//     std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const;

// private:
//     amp::LinkManipulator m_manipulator;
//     std::vector<amp::Obstacle2D> m_obstacles;
//     tools m_tools; 
//     std::vector<std::vector<int>> cSpaceGrid;

//     double m_x0_min;
//     double m_x0_max;
//     double m_x1_min;
//     double m_x1_max;
//     std::size_t m_x0_cells;
//     std::size_t m_x1_cells;
// };

// class MyGridCSpace2DConstructor: public amp::GridCSpace2DConstructor{
// public:
//     MyGridCSpace2DConstructor(std::size_t x0_cells, std::size_t x1_cells, 
//                             double x0_min, double x0_max, 
//                             double x1_min, double x1_max)
//                             : m_x0_min(x0_min),
//                              m_x0_max(x0_max),
//                             m_x1_min(x1_min),
//                             m_x1_max(x1_max),
//                             m_x0_cells(x0_cells),
//                             m_x1_cells(x1_cells)
//         {
            
//         }

//     std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env){
//         amp::LinkManipulator MyManipulator(manipulator.getLinkLengths());
//         std::unique_ptr<MyGridCSpace> cSpace = std::make_unique<MyGridCSpace>(m_x0_cells, 
//                                                                           m_x1_cells, 
//                                                                           m_x0_min, 
//                                                                           m_x0_max, 
//                                                                           m_x1_min, 
//                                                                           m_x1_max, 
//                                                                           MyManipulator, 
//                                                                           env.obstacles);
//         return cSpace;
//     }
// private:
//     double m_x0_min;
//     double m_x0_max;
//     double m_x1_min;
//     double m_x1_max;
//     std::size_t m_x0_cells;
//     std::size_t m_x1_cells;
   
// };