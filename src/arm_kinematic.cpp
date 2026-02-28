#include "arm_mazzolini/arm_kinematic.hpp"

namespace arm_mazzolini {

    ArmKinematic::ArmKinematic(double l1, double l2)
        : l1_(l1), l2_(l2)
    {
        // Do somenthing here?
    }

    // TODO: add elbow up and exclusion zones
    bool ArmKinematic::computeIK(const Eigen::Vector3d& position, std::vector<double>& theta, ErrorType& error_type)
    {
        if(position.isZero()) {
            // TODO: Modficare questo check, così non va bene
            error_type = ErrorType::TARGET_EMPTY;
            return false;
        }
        double x = position.x();
        double y = position.y();
        double r = std::sqrt(x*x + y*y);

        // Check for errors
        if (x < 0 || r < std::abs(l1_ - l2_)) {
            error_type = ErrorType::EXCLUSION_ZONE;
            return false;
        }
        else if (r > (l1_ + l2_) ) {
            error_type = ErrorType::TARGET_TOO_FAR;
            return false;
        }
        else {
            double cos_theta2 = (r*r - l1_*l1_ - l2_*l2_) / (2*l1_*l2_);

            // Numerical safety for arccos
            if (cos_theta2 > 1.0) cos_theta2 = 1.0;
            else if (cos_theta2 < -1.0) cos_theta2 = -1.0;

            double sin_theta2 = std::sqrt(1 - cos_theta2 * cos_theta2); // elbow_down solution
            double theta2 = std::atan2(sin_theta2, cos_theta2);

            double k1 = l1_ + l2_ * cos_theta2;
            double k2 = l2_ * sin_theta2;
            double theta1 = std::atan2(y, x) - std::atan2(k2, k1);

            // Normalize angles
            theta1 = normalizeAngle(theta1);
            theta2 = normalizeAngle(theta2);
            theta = {theta1, theta2};

            return true;
        }
    }

    bool ArmKinematic::computeFK(const std::vector<double>& theta,
                             Eigen::Vector3d& position,
                             ErrorType& error_type)
    {
        // Check input validity
        if (theta.size() != 2) {
            error_type = ErrorType::TARGET_EMPTY;
            return false;
        }

        double theta1 = theta[0];
        double theta2 = theta[1];

        // Forward kinematics for planar RR arm
        double x = l1_ * std::cos(theta1)
                + l2_ * std::cos(theta1 + theta2);

        double y = l1_ * std::sin(theta1)
                + l2_ * std::sin(theta1 + theta2);

        position.x() = x;
        position.y() = y;
        position.z() = 0.0;

        return true;
    }

    double ArmKinematic::normalizeAngle(double angle)
    {
        while (angle < -M_PI) angle += 2 * M_PI;
        while (angle > M_PI) angle -= 2 * M_PI;
        return angle;
    }

    Eigen::Matrix2d ArmKinematic::computeJacobian(const std::vector<double>& theta)
    {
        double theta1 = theta[0];
        double theta2 = theta[1];

        double s1  = std::sin(theta1);
        double c1  = std::cos(theta1);
        double s12 = std::sin(theta1 + theta2);
        double c12 = std::cos(theta1 + theta2);

        Eigen::Matrix2d J;

        J << - l1_ * s1 - l2_* s12,   -l2_ * s12, 
            l1_ * c1 + l2_ * c12,    l2_ * c12;

        return J;
    }

    Eigen::Matrix3d ArmKinematic::computeScaraJacobian(const std::vector<double>& theta)
    {
        // this function provides Jacobian with an additional row for z-axis

        Eigen::Matrix2d J_planar = computeJacobian(theta);
        Eigen::Matrix3d J_scara = Eigen::Matrix3d::Zero();
        J_scara.block<2, 2>(0, 0) = J_planar;
        J_scara(2, 2) = 1.0;
        return J_scara;
    }

} // namespace arm_mazzolini