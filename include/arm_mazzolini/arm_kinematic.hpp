#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <string>
#include "arm_mazzolini/shared_classes.hpp"

namespace arm_mazzolini {
    
    class ArmKinematic
    {
    public:
        ArmKinematic(double l1, double l2);
        bool computeIK(const Eigen::Vector3d& position,std::vector<double>& theta, ErrorType& error_type);
        bool computeFK(const std::vector<double>& theta, Eigen::Vector3d& position, ErrorType& error_type);
        double normalizeAngle(double angle);
        Eigen::Matrix2d computeJacobian(const std::vector<double>& theta);
        Eigen::Matrix3d computeScaraJacobian(const std::vector<double>& theta);

        private:
        double l1_;
        double l2_;

    };
} // namespace arm_mazzolini