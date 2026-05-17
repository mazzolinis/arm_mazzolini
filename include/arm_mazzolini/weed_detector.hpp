#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>

#include <onnxruntime_cxx_api.h>

#include <string>
#include <vector>
#include <memory>

#include "arm_mazzolini/shared_classes.hpp"

namespace arm_mazzolini
{

// Detection result for a single instance
struct WeedDetection
{
    int class_id;           // 0 = weed_1, 1 = weed_2
    float confidence;
    cv::Rect bbox;
    cv::Mat mask;           // binary mask, same size as input image
    cv::Point centroid;     // pixel centroid of the mask
};

class WeedDetector
{
public:
    /**
     * @param model_path     Path to the exported YOLO11s-seg ONNX model
     * @param conf_threshold Minimum confidence to accept a detection
     * @param depth_roi_size Side (pixels) of the square ROI used to compute median depth
     * @param device_id      CUDA device index (0 = first GPU)
     */
    WeedDetector(
        const std::string &model_path,
        float conf_threshold  = 0.25f,
        int   depth_roi_size  = 20,
        int   device_id       = 0);

    ~WeedDetector() = default;

    // ── Camera calibration ─────────────────────────────────────────────────
    void SetCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info_msg);
    std::vector<double> getCameraCenter();
    std::vector<double> getCameraFocals();

    // ── Position-Based control (metric 3-D position) ───────────────────────
    /**
     * Returns the 3-D centroid of the HIGHEST-confidence detection.
     * centroid_position : (X, Y, Z) in metres in the camera frame.
     * Returns false when no weed is found.
     */
    bool PBTargetPosition(
        const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
        Eigen::Vector3d &centroid_position);

    /**
     * Overload: convert an already-computed image-based feature to metric.
     * feature = (x_n, y_n, Z) where x_n = (u-cx)/fx
     */
    Eigen::Vector3d PBTargetPosition(const Eigen::Vector3d &centroid_feature);

    // ── Image-Based control (normalised coordinates) ───────────────────────
    /**
     * centroid_feature : ((u-cx)/fx, (v-cy)/fy, Z)
     * Returns false when no weed is found.
     */
    bool IBTargetPosition(
        const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
        Eigen::Vector3d &centroid_feature);

    // ── Extended API: access all detections ───────────────────────────────
    /**
     * Runs full inference and returns all detections above threshold.
     * Useful for publishing per-class markers, visualisation, etc.
     */
    bool RGB_analysis(
        cv::Mat &rgb,
        std::vector<WeedDetection> &detections);

private:
    // ── ONNX Runtime state ─────────────────────────────────────────────────
    Ort::Env                              ort_env_;
    Ort::SessionOptions                   session_options_;
    std::unique_ptr<Ort::Session>         session_;
    Ort::AllocatorWithDefaultOptions      allocator_;

    // Model I/O metadata
    std::vector<std::string>              input_names_str_;
    std::vector<std::string>              output_names_str_;
    std::vector<const char*>              input_names_;
    std::vector<const char*>              output_names_;
    int64_t                               model_input_w_{640};
    int64_t                               model_input_h_{640};

    // ── Parameters ────────────────────────────────────────────────────────
    float  conf_threshold_;
    float  iou_threshold_{0.45f};
    int    depth_roi_size_;
    int    num_classes_{2};           // weed_1, weed_2
    int    num_masks_{32};            // YOLO seg prototype count

    // ── Camera intrinsics ─────────────────────────────────────────────────
    double fx_{1.0}, fy_{1.0}, cx_{0.0}, cy_{0.0};

    // ── Internal helpers ──────────────────────────────────────────────────

    /**
     * Pre-process: resize + pad to model_input_w_ x model_input_h_,
     * convert to CHW float32 normalised [0,1].
     * Returns the scale and padding applied (needed to map boxes back).
     */
    cv::Mat preprocess(const cv::Mat &rgb,
                       float &scale,
                       int &pad_w, int &pad_h) const;

    /**
     * Post-process raw ONNX outputs into WeedDetection objects.
     * Handles NMS, mask decoding and back-projection to original image size.
     */
    std::vector<WeedDetection> postprocess(
        const std::vector<Ort::Value> &outputs,
        int orig_w, int orig_h,
        float scale, int pad_w, int pad_h) const;

    /** Non-Maximum Suppression (class-agnostic). */
    std::vector<int> nms(
        const std::vector<cv::Rect2f> &boxes,
        const std::vector<float>      &scores,
        float                          iou_threshold) const;

    /** Decode a single instance mask from prototypes + coefficients. */
    cv::Mat decodeMask(
        const float*   proto_data,    // (num_masks, mH, mW) flat
        int            mH, int mW,
        const float*   coeff,         // num_masks coefficients
        const cv::Rect &bbox_on_input,// bbox in 640x640 space (for crop)
        int orig_w, int orig_h,
        float scale, int pad_w, int pad_h) const;

    /** Compute pixel centroid of a binary mask. */
    cv::Point maskCentroid(const cv::Mat &mask) const;

    /** Median depth in a square ROI centred on a pixel. */
    double getDepthMedian(const cv::Mat &depth, const cv::Point &center) const;

    /**
     * Full detect: decode image → run ONNX → postprocess.
     * Returns false if cv_bridge conversion fails or no detection found.
     */
    bool detect(
        const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
        cv::Point &best_centroid,
        double    &Z,
        int       *best_class_id = nullptr);
};

} // namespace arm_mazzolini
