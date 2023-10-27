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

    if (nLinks() == 2){
        std::cout << "2 link" << std::endl;
        Eigen::VectorXd theta(2);

        double L1 = getLinkLengths()[0];
        double L2 = getLinkLengths()[1];

        double x = end_effector_location(0);
        double y = end_effector_location(1);       
        double r = sqrt(x*x + y*y);

        std::cout << "end effector: (" << x << ", " << y << ")" << std::endl;
        std::cout << "links: (" << L1 << ", " << L2 << ")" << std::endl;

        // check that possition is reachable
        if (r > L1 + L2 || r < std::abs(L1 - L2)) {
            std::cerr << "The desired position is unreachable." << std::endl;
            theta << NAN, NAN;
            return theta;
        }

        double theta2 = std::acos((x*x + y*y - L1*L1 - L2*L2) / (2 * L1 * L2));
        double alpha = std::asin(L2 * std::sin(theta2) / r);
        double beta = std::atan2(y, x);

        theta(0) = beta + alpha; // theta1
        theta(1) = theta2; // theta2

        return theta;
    }

    Eigen::VectorXd theta = Eigen::VectorXd::Constant(nLinks(), M_PI/4.0);  // initial guess

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
        ManipulatorState dTheta = resultEigen;

        for (std::size_t i = 0; i < nLinks(); ++i) {
            theta[i] += dTheta[i];

            // Normalize the angle to be within [0, 2pi)
            theta[i] = std::fmod(theta[i], 2 * M_PI);
            if (theta[i] < 0) {
                theta[i] += 2 * M_PI;
            }
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

} // namespace amp

