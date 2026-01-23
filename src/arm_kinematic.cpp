#include "arm_mazzolini/arm_kinematic.hpp"

namespace arm_mazzolini {

    ArmKinematic::ArmKinematic(double l1, double l2)
        : l1_(l1), l2_(l2)
    {
        // Do somenthing here?
    }

    // TODO: add elbow up and exclusion zones
    bool ArmKinematic::computeIK(const Eigen::Vector3d& position, std::vector<double> theta, ErrorType& error_type)
    {
        if(theta.empty()) {
            error_type = ErrorType::EXCLUSION_ZONE;
            return false;
        }
        double x = position.x();
        double y = position.y();
        double r = std::sqrt(x*x + y*y);

        // Check for errors
        if (x < 0) {
            error_type = ErrorType::EXCLUSION_ZONE;
            return false;
        }
        else if (r > (l1_+l2_) || r < std::abs(l1_-l2_)) {
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

    double ArmKinematic::normalizeAngle(double angle)
    {
        while (angle < -M_PI) angle += 2 * M_PI;
        while (angle > M_PI) angle -= 2 * M_PI;
        return angle;
    }

} // namespace arm_mazzolini