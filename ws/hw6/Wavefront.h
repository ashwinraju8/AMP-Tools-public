#include "AMPCore.h"
#include "hw/HW6.h"
#include "cSpace.h"
#include "DerivedLinkManipulator.h"
#include "HelpfulClass.h"
#include <queue>

class MyPointWFAlgo : public amp::PointWaveFrontAlgorithm {
    public:
        virtual std::unique_ptr<amp::GridCSpace2D> constructDiscretizedWorkspace(const amp::Environment2D& environment) override;

        // This is just to get grade to work, you DO NOT need to override this method
        // virtual amp::Path2D plan(const amp::Problem2D& problem) override {
        //     return amp::Path2D();
        // }

        // You need to implement here
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override;


};

class MyManipWFAlgo : public amp::ManipulatorWaveFrontAlgorithm {
    public:
        // Default ctor
        MyManipWFAlgo(
            const amp::LinkManipulator& manipulator, 
            std::shared_ptr<amp::GridCSpace2DConstructor> cspaceConstructor)
            : amp::ManipulatorWaveFrontAlgorithm(cspaceConstructor), 
                                                manipulator(manipulator) 
        {
        }

        // You can have custom ctor params for all of these classes
        // MyManipWFAlgo(const amp::LinkManipulator& manipulator, 
        //             std::size_t x0_cells, std::size_t x1_cells, 
        //             double x0_min, double x0_max, 
        //             double x1_min, double x1_max)
        //     : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyGridCSpace2DConstructor>(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max)), 
        //     manipulator(manipulator) 
        // {

        // }

    //     MyManipWFAlgo(const amp::LinkManipulator& manipulator, 
    //               MyGridCSpace2DConstructor cspaceConstructor)
    //     : amp::ManipulatorWaveFrontAlgorithm(std::make_shared<MyGridCSpace2DConstructor>(cspaceConstructor)), 
    //       manipulator(manipulator) 
    // {
    // }



        // // This is just to get grade to work, you DO NOT need to override this method
        // amp::ManipulatorTrajectory2Link plan(const amp::LinkManipulator2D& link_manipulator_agent, const amp::Problem2D& problem){
        //     return amp::ManipulatorTrajectory2Link();
        // }
        
        // You need to implement here
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace) override;
    private:
        amp::LinkManipulator manipulator; // The manipulator member variable
};