#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <string>

namespace arm_mazzolini {
    
    class armKinematic
    {
    public:
        armKinematic(double l1, double l2);
        int computeIK(const Eigen::Vector3d& position,std::vector<double> theta);
        double normalizeAngle(double angle);

        private:
        double l1_;
        double l2_;

    };
} // namespace arm_mazzolini