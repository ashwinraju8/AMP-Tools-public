#pragma once
#include "AMPCore.h"
#include <Eigen/LU>
#include <Eigen/SVD>

namespace amp {
    
class LinkManipulator : public LinkManipulator2D {
    public:
        using ManipulatorState = amp::ManipulatorState;
        using LinkManipulator2D::LinkManipulator2D;  // Inherit constructors

        Eigen::Vector2d getJointLocation(const ManipulatorState& state, uint32_t joint_index) const override;

        Eigen::MatrixXd computeJacobian(const ManipulatorState& theta) const;
        ManipulatorState getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const override;
};

}