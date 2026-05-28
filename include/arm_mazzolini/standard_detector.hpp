#ifndef ARM_MAZZOLINI__STANDARD_DETECTOR_HPP_
#define ARM_MAZZOLINI__STANDARD_DETECTOR_HPP_

#include <vector>

#include <Eigen/Dense>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace arm_mazzolini
{
    /**
     *  Abstract interface for target detectors used by WeederController.
     *
     *  Concrete implementations (e.g. SphereDetector using Excess Green Index,
     *  WeedDetector using a YOLO segmentation network) must derive from this
     *  class and implement every pure virtual method.
     *
     *  The interface deliberately mirrors the public API that both existing
     *  detectors already expose, so no behavioural change is required in the
     *  derived classes — only the inheritance declaration and the `override`
     *  keyword on the corresponding methods.
     */
    class StandardDetector
    {
    public:
        // Virtual destructor: mandatory in a polymorphic base class.
        // Without it, deleting a derived object through a StandardDetector*
        // (e.g. when std::unique_ptr<StandardDetector> goes out of scope)
        // is undefined behaviour.
        virtual ~StandardDetector() = default;

        // ─────────────────────────────────────────────────────────────────
        // Camera calibration
        // ─────────────────────────────────────────────────────────────────

        /// Store intrinsic parameters (fx, fy, cx, cy) from a CameraInfo msg.
        virtual void SetCameraInfo(
            const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info_msg) = 0;

        /// Returns {cx, cy} — principal point in pixels.
        virtual std::vector<double> getCameraCenter() = 0;

        /// Returns {fx, fy} — focal lengths in pixels.
        virtual std::vector<double> getCameraFocals() = 0;

        // ─────────────────────────────────────────────────────────────────
        // Detection — image-based (IB) and position-based (PB)
        // ─────────────────────────────────────────────────────────────────

        /**
         *  Image-based target feature.
         *  Output: normalized pixel coordinates relative to the principal
         *  point ((u - cx)/fx, (v - cy)/fy), with depth Z (metres) in z().
         *  Returns false if no target is detected.
         */
        virtual bool IBTargetPosition(
            const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
            const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
            Eigen::Vector3d &centroid_feature) = 0;

        /**
         *  Position-based target position (camera frame, metres).
         *  Returns false if no target is detected.
         */
        virtual bool PBTargetPosition(
            const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
            const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
            Eigen::Vector3d &centroid_position) = 0;

        /**
         *  Convert an already-computed feature (normalized coords + Z)
         *  into a metric position in the camera frame. Pure geometry,
         *  no image reprocessing.
         */
        virtual Eigen::Vector3d PBTargetPosition(
            const Eigen::Vector3d &centroid_feature) = 0;
    };

} // namespace arm_mazzolini

#endif // ARM_MAZZOLINI__STANDARD_DETECTOR_HPP_
