#include "DerivedLinkManipulator.h"


namespace amp {
    

Eigen::Vector2d LinkManipulator::getJointLocation(const ManipulatorState& state, uint32_t joint_index) const {
    if (joint_index == 0) {
        return m_base_location;
    }

    Eigen::Vector2d joint_location = m_base_location;
    double cumulative_theta = 0.0;

    for (uint32_t i = 0; i < joint_index && i < state.size(); ++i) {
        cumulative_theta += state[i];
        joint_location[0] += m_link_lengths[i] * std::cos(cumulative_theta);
        joint_location[1] += m_link_lengths[i] * std::sin(cumulative_theta);
    }

    return joint_location;
}


Eigen::MatrixXd LinkManipulator::computeJacobian(const ManipulatorState& theta) const {
    Eigen::MatrixXd jacobian(2, nLinks());
    for (std::size_t i = 0; i < nLinks(); ++i) {
        Eigen::Vector2d joint = getJointLocation(theta, i);
        Eigen::Vector2d end_effector = getJointLocation(theta, nLinks());
        Eigen::Vector2d diff = end_effector - joint;

        jacobian(0, i) = -diff[1];
        jacobian(1, i) = diff[0];
    }

    return jacobian;
}
ManipulatorState LinkManipulator::getConfigurationFromIK(const Eigen::Vector2d& end_effector_location) const {
    ManipulatorState theta(nLinks(), M_PI/4.0);  // initial guess

    const double alpha = 0.01;  // learning rate (optimization parameter)
    const double tolerance = 1e-4;  // error tolerance
    const int idxMax = 1000;  // maximum iterations
    int idx = 0;
    

    while (idx < idxMax) {
        Eigen::Vector2d current_end_effector = getJointLocation(theta, nLinks());
        Eigen::Vector2d error = end_effector_location - current_end_effector;
        if (error.norm() < tolerance) {
            break;
        }

        Eigen::MatrixXd jacobian = computeJacobian(theta);
        // std::cout << "Jacobian is:\n" << jacobian << std::endl;

        // Compute the pseudo-inverse using Singular Value Decomposition (SVD)
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::VectorXd singularValues = svd.singularValues();

        double threshold = 1e-6;  // Threshold for singularity
        for (int i = 0; i < singularValues.size(); ++i) {
            if (singularValues[i] < threshold) {
                singularValues[i] = 0.0;
            } else {
                singularValues[i] = 1.0 / singularValues[i];
            }
        }
        Eigen::MatrixXd pseudoInverse = svd.matrixV() * singularValues.asDiagonal() * svd.matrixU().adjoint();

        Eigen::VectorXd resultEigen = alpha * pseudoInverse * error;
        ManipulatorState dTheta(resultEigen.data(), resultEigen.data() + resultEigen.size());

        for (std::size_t i = 0; i < nLinks(); ++i) {
            theta[i] += dTheta[i];
        }

        // std::cout << "error is: " << error.norm() << std::endl;
        // std::cout << "Theta at iteration " << idx << ": ";
        // for (const auto &val : theta) {
        //     std::cout << val << " ";
        // }
        // std::cout << std::endl;
        // std::cout << "Jacobian is: \n" << jacobian << std::endl;
        // std::cout << "Pseudo Inverse is: \n" << pseudoInverse << std::endl;

        idx++;
    }

    return theta;
}

}

