#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <string>
#include "arm_mazzolini/error_type.hpp"

namespace arm_mazzolini {
    
    class ArmKinematic
    {
    public:
        ArmKinematic(double l1, double l2);
        bool computeIK(const Eigen::Vector3d& position,std::vector<double> theta, ErrorType& error_type);
        double normalizeAngle(double angle);

        private:
        double l1_;
        double l2_;

    };
} // namespace arm_mazzolini