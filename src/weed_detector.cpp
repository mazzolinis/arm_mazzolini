#include "arm_mazzolini/sphere_detector.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace arm_mazzolini
{

// ─────────────────────────────────────────────────────────────────────────────
// Constructor
// ─────────────────────────────────────────────────────────────────────────────

WeedDetector::WeedDetector(
    const std::string &model_path,
    float conf_threshold,
    int   depth_roi_size,
    int   device_id)
    : ort_env_(ORT_LOGGING_LEVEL_WARNING, "YOLOWeedDetector"),
      conf_threshold_(conf_threshold),
      depth_roi_size_(depth_roi_size)
{
    // ── Session options ───────────────────────────────────────────────────
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);

    // ── CUDA execution provider ───────────────────────────────────────────
    OrtCUDAProviderOptions cuda_opts;
    cuda_opts.device_id = device_id;
    session_options_.AppendExecutionProvider_CUDA(cuda_opts);

    // ── Load model ────────────────────────────────────────────────────────
    session_ = std::make_unique<Ort::Session>(ort_env_, model_path.c_str(), session_options_);

    // ── Collect I/O names ─────────────────────────────────────────────────
    const size_t num_inputs = session_->GetInputCount();
    for (size_t i = 0; i < num_inputs; ++i)
    {
        auto name = session_->GetInputNameAllocated(i, allocator_);
        input_names_str_.emplace_back(name.get());
    }

    const size_t num_outputs = session_->GetOutputCount();
    for (size_t i = 0; i < num_outputs; ++i)
    {
        auto name = session_->GetOutputNameAllocated(i, allocator_);
        output_names_str_.emplace_back(name.get());
    }

    // Build raw-pointer vectors expected by Run()
    for (const auto &n : input_names_str_)  input_names_.push_back(n.c_str());
    for (const auto &n : output_names_str_) output_names_.push_back(n.c_str());

    // ── Read input spatial dimensions from model metadata ─────────────────
    auto input_shape = session_->GetInputTypeInfo(0)
                           .GetTensorTypeAndShapeInfo()
                           .GetShape();
    // shape is [batch, channels, H, W]
    if (input_shape.size() == 4 && input_shape[2] > 0 && input_shape[3] > 0)
    {
        model_input_h_ = input_shape[2];
        model_input_w_ = input_shape[3];
    }

    RCLCPP_INFO(rclcpp::get_logger("WeedDetector"),
                "YOLO weed detector loaded: %s  input=%ldx%ld  CUDA device=%d",
                model_path.c_str(), model_input_w_, model_input_h_, device_id);
}

// ─────────────────────────────────────────────────────────────────────────────
// Camera calibration
// ─────────────────────────────────────────────────────────────────────────────

void WeedDetector::SetCameraInfo(
    const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info_msg)
{
    fx_ = info_msg->k[0];
    fy_ = info_msg->k[4];
    cx_ = info_msg->k[2];
    cy_ = info_msg->k[5];
}

std::vector<double> WeedDetector::getCameraCenter() { return {cx_, cy_}; }
std::vector<double> WeedDetector::getCameraFocals()  { return {fx_, fy_}; }


bool WeedDetector::IBTargetPosition(
    const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
    Eigen::Vector3d &centroid_feature)
{
    cv::Point centroid;
    double Z;
    if (!detect(rgb_msg, depth_msg, centroid, Z))
        return false;

    centroid_feature.x() = (centroid.x - cx_) / fx_;
    centroid_feature.y() = (centroid.y - cy_) / fy_;
    centroid_feature.z() = Z;
    return true;
}

bool WeedDetector::PBTargetPosition(
    const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
    Eigen::Vector3d &centroid_position)
{
    cv::Point centroid;
    double Z;
    if (!detect(rgb_msg, depth_msg, centroid, Z))
        return false;

    centroid_position.x() = (centroid.x - cx_) * Z / fx_;
    centroid_position.y() = (centroid.y - cy_) * Z / fy_;
    centroid_position.z() = Z;
    return true;
}

Eigen::Vector3d WeedDetector::PBTargetPosition(const Eigen::Vector3d &centroid_feature)
{
    Eigen::Vector3d pos;
    pos.x() = centroid_feature.x() * centroid_feature.z();
    pos.y() = centroid_feature.y() * centroid_feature.z();
    pos.z() = centroid_feature.z();
    return pos;
}

// ─────────────────────────────────────────────────────────────────────────────
// Extended API: all detections
// ─────────────────────────────────────────────────────────────────────────────

bool WeedDetector::DetectAll(
    const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
    std::vector<WeedDetection> &detections)
{
    // ── Decode RGB ────────────────────────────────────────────────────────
    cv::Mat rgb;
    try
    {
        rgb = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::RGB8)->image;
    }
    catch (cv_bridge::Exception &ex)
    {
        RCLCPP_ERROR(rclcpp::get_logger("WeedDetector"),
                     "cv_bridge RGB exception: %s", ex.what());
        return false;
    }

    // ── Decode Depth ──────────────────────────────────────────────────────
    cv::Mat depth;
    try
    {
        depth = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding)->image;
    }
    catch (cv_bridge::Exception &ex)
    {
        RCLCPP_ERROR(rclcpp::get_logger("WeedDetector"),
                     "cv_bridge Depth exception: %s", ex.what());
        return false;
    }

    const int orig_w = rgb.cols;
    const int orig_h = rgb.rows;

    // ── Pre-process ───────────────────────────────────────────────────────
    float scale;
    int pad_w, pad_h;
    cv::Mat blob = preprocess(rgb, scale, pad_w, pad_h);

    // ── Build input tensor ────────────────────────────────────────────────
    std::vector<int64_t> input_shape = {1, 3, model_input_h_, model_input_w_};
    Ort::MemoryInfo mem_info = Ort::MemoryInfo::CreateCpu(
        OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        mem_info,
        reinterpret_cast<float*>(blob.data),
        static_cast<size_t>(1 * 3 * model_input_h_ * model_input_w_),
        input_shape.data(), input_shape.size());

    // ── Inference ─────────────────────────────────────────────────────────
    auto outputs = session_->Run(
        Ort::RunOptions{nullptr},
        input_names_.data(),  &input_tensor, 1,
        output_names_.data(), output_names_.size());

    // ── Post-process ──────────────────────────────────────────────────────
    detections = postprocess(outputs, orig_w, orig_h, scale, pad_w, pad_h);

    // ── Add depth to each detection ───────────────────────────────────────
    // (getDepthMedian is robust: returns 0 on failure)
    for (auto &det : detections)
    {
        det.centroid = maskCentroid(det.mask);
        // depth stored externally by the caller if needed; no Z field in
        // WeedDetection by design — callers use getDepthMedian themselves
        // or use detect() which wraps this.
    }

    return !detections.empty();
}

// ─────────────────────────────────────────────────────────────────────────────
// Internal: unified detect (used by IB / PB)
// ─────────────────────────────────────────────────────────────────────────────

bool WeedDetector::detect(
    const sensor_msgs::msg::Image::ConstSharedPtr &rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr &depth_msg,
    cv::Point &best_centroid,
    double    &Z,
    int       *best_class_id)
{
    std::vector<WeedDetection> detections;

    // ── Decode Depth (needed here for Z) ──────────────────────────────────
    cv::Mat depth;
    try
    {
        depth = cv_bridge::toCvCopy(depth_msg, depth_msg->encoding)->image;
    }
    catch (cv_bridge::Exception &ex)
    {
        RCLCPP_ERROR(rclcpp::get_logger("WeedDetector"),
                     "cv_bridge Depth exception: %s", ex.what());
        return false;
    }

    // Re-use DetectAll for RGB inference
    sensor_msgs::msg::Image::ConstSharedPtr dummy_depth = depth_msg; // not used inside
    if (!DetectAll(rgb_msg, depth_msg, detections))
        return false;

    // ── Pick best detection (highest confidence) ──────────────────────────
    const WeedDetection *best = nullptr;
    for (const auto &d : detections)
    {
        if (!best || d.confidence > best->confidence)
            best = &d;
    }

    best_centroid = best->centroid;
    Z = getDepthMedian(depth, best_centroid);

    if (best_class_id)
        *best_class_id = best->class_id;

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Pre-processing: letterbox resize → CHW float blob
// ─────────────────────────────────────────────────────────────────────────────

cv::Mat WeedDetector::preprocess(
    const cv::Mat &rgb,
    float &scale,
    int &pad_w, int &pad_h) const
{
    const int iw = model_input_w_;
    const int ih = model_input_h_;

    scale = std::min(static_cast<float>(iw) / rgb.cols,
                     static_cast<float>(ih) / rgb.rows);

    const int new_w = static_cast<int>(std::round(rgb.cols * scale));
    const int new_h = static_cast<int>(std::round(rgb.rows * scale));

    pad_w = (iw - new_w) / 2;
    pad_h = (ih - new_h) / 2;

    cv::Mat resized;
    cv::resize(rgb, resized, cv::Size(new_w, new_h), 0, 0, cv::INTER_LINEAR);

    // Letterbox with grey padding (value 114 — Ultralytics default)
    cv::Mat letterboxed(ih, iw, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(letterboxed(cv::Rect(pad_w, pad_h, new_w, new_h)));

    // HWC uint8 → CHW float32 [0,1]
    cv::Mat float_img;
    letterboxed.convertTo(float_img, CV_32FC3, 1.0 / 255.0);

    // Interleaved → planar
    const int area = iw * ih;
    cv::Mat blob(1, 3 * area, CV_32F);
    float *data = reinterpret_cast<float*>(blob.data);
    std::vector<cv::Mat> planes(3);
    cv::split(float_img, planes);
    std::memcpy(data,             planes[0].data, area * sizeof(float));
    std::memcpy(data + area,      planes[1].data, area * sizeof(float));
    std::memcpy(data + 2 * area,  planes[2].data, area * sizeof(float));

    return blob;
}

// ─────────────────────────────────────────────────────────────────────────────
// Post-processing
// ─────────────────────────────────────────────────────────────────────────────
/*
 * YOLO11-seg ONNX exports two outputs (when dynamic=False, imgsz=640):
 *   output0  shape [1, 4+num_classes+num_masks, num_anchors]  — detection head
 *   output1  shape [1, num_masks, mH, mW]                     — mask prototypes
 *
 * Each anchor: [x_c, y_c, w, h, cls0_score, …, clsN_score, m0, …, m31]
 */

std::vector<WeedDetection> WeedDetector::postprocess(
    const std::vector<Ort::Value> &outputs,
    int orig_w, int orig_h,
    float scale, int pad_w, int pad_h) const
{
    // ── Unpack tensors ────────────────────────────────────────────────────
    const float *det_data  = outputs[0].GetTensorData<float>();
    const float *proto_data = outputs[1].GetTensorData<float>();

    auto det_shape   = outputs[0].GetTensorTypeAndShapeInfo().GetShape(); // [1, rows, anchors]
    auto proto_shape = outputs[1].GetTensorTypeAndShapeInfo().GetShape(); // [1, nm, mH, mW]

    // det_shape: [1, num_rows, num_anchors]
    const int num_anchors = static_cast<int>(det_shape[2]);
    const int num_rows    = static_cast<int>(det_shape[1]); // 4 + nc + nm
    const int mH          = static_cast<int>(proto_shape[2]);
    const int mW          = static_cast<int>(proto_shape[3]);

    // ── Parse detections ──────────────────────────────────────────────────
    std::vector<cv::Rect2f> boxes;
    std::vector<float>      scores;
    std::vector<int>        class_ids;
    std::vector<int>        anchor_indices;

    for (int a = 0; a < num_anchors; ++a)
    {
        // Column-major layout: element [row, anchor] = det_data[row * num_anchors + a]
        const float xc = det_data[0 * num_anchors + a];
        const float yc = det_data[1 * num_anchors + a];
        const float w  = det_data[2 * num_anchors + a];
        const float h  = det_data[3 * num_anchors + a];

        // Find best class
        float best_score = -1.f;
        int   best_cls   = -1;
        for (int c = 0; c < num_classes_; ++c)
        {
            float s = det_data[(4 + c) * num_anchors + a];
            if (s > best_score) { best_score = s; best_cls = c; }
        }

        if (best_score < conf_threshold_)
            continue;

        // cx,cy,w,h are in the 640x640 letterboxed space
        cv::Rect2f box(xc - w / 2.f, yc - h / 2.f, w, h);
        boxes.push_back(box);
        scores.push_back(best_score);
        class_ids.push_back(best_cls);
        anchor_indices.push_back(a);
    }

    if (boxes.empty())
        return {};

    // ── NMS ───────────────────────────────────────────────────────────────
    std::vector<int> keep = nms(boxes, scores, iou_threshold_);

    // ── Build WeedDetection objects ───────────────────────────────────────
    std::vector<WeedDetection> results;
    results.reserve(keep.size());

    for (int k : keep)
    {
        WeedDetection det;
        det.class_id   = class_ids[k];
        det.confidence = scores[k];

        // ── Back-project bbox to original image space ─────────────────────
        cv::Rect2f lb_box = boxes[k];
        float x1 = (lb_box.x - pad_w) / scale;
        float y1 = (lb_box.y - pad_h) / scale;
        float x2 = (lb_box.x + lb_box.width  - pad_w) / scale;
        float y2 = (lb_box.y + lb_box.height - pad_h) / scale;

        x1 = std::clamp(x1, 0.f, static_cast<float>(orig_w - 1));
        y1 = std::clamp(y1, 0.f, static_cast<float>(orig_h - 1));
        x2 = std::clamp(x2, 0.f, static_cast<float>(orig_w - 1));
        y2 = std::clamp(y2, 0.f, static_cast<float>(orig_h - 1));

        det.bbox = cv::Rect(static_cast<int>(x1), static_cast<int>(y1),
                            static_cast<int>(x2 - x1), static_cast<int>(y2 - y1));

        // ── Decode mask ───────────────────────────────────────────────────
        const int a = anchor_indices[k];
        const float *coeff = det_data + (4 + num_classes_) * num_anchors + a;
        // coeff points to the first mask coefficient of this anchor
        // (layout: rows for each mask coeff are contiguous in column-major)
        // We need to gather the num_masks_ coefficients for anchor a:
        std::vector<float> coeff_vec(num_masks_);
        for (int m = 0; m < num_masks_; ++m)
            coeff_vec[m] = det_data[(4 + num_classes_ + m) * num_anchors + a];

        det.mask = decodeMask(
            proto_data, mH, mW,
            coeff_vec.data(),
            cv::Rect(static_cast<int>(lb_box.x), static_cast<int>(lb_box.y),
                     static_cast<int>(lb_box.width), static_cast<int>(lb_box.height)),
            orig_w, orig_h, scale, pad_w, pad_h);

        det.centroid = maskCentroid(det.mask);
        results.push_back(std::move(det));
    }

    return results;
}

// ─────────────────────────────────────────────────────────────────────────────
// NMS
// ─────────────────────────────────────────────────────────────────────────────

std::vector<int> WeedDetector::nms(
    const std::vector<cv::Rect2f> &boxes,
    const std::vector<float>      &scores,
    float                          iou_threshold) const
{
    std::vector<int> indices(boxes.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(),
              [&scores](int a, int b){ return scores[a] > scores[b]; });

    std::vector<int> keep;
    std::vector<bool> suppressed(boxes.size(), false);

    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        if (suppressed[idx]) continue;
        keep.push_back(idx);

        const cv::Rect2f &a = boxes[idx];
        for (size_t j = i + 1; j < indices.size(); ++j)
        {
            int jdx = indices[j];
            if (suppressed[jdx]) continue;
            const cv::Rect2f &b = boxes[jdx];

            float ix1 = std::max(a.x, b.x);
            float iy1 = std::max(a.y, b.y);
            float ix2 = std::min(a.x + a.width,  b.x + b.width);
            float iy2 = std::min(a.y + a.height, b.y + b.height);

            float inter = std::max(0.f, ix2 - ix1) * std::max(0.f, iy2 - iy1);
            float uni   = a.area() + b.area() - inter;
            if (uni > 0.f && inter / uni > iou_threshold)
                suppressed[jdx] = true;
        }
    }
    return keep;
}

// ─────────────────────────────────────────────────────────────────────────────
// Mask decoding
// ─────────────────────────────────────────────────────────────────────────────

cv::Mat WeedDetector::decodeMask(
    const float*   proto_data,
    int            mH, int mW,
    const float*   coeff,
    const cv::Rect &bbox_lb,     // bbox in letterboxed 640x640 space
    int orig_w, int orig_h,
    float scale, int pad_w, int pad_h) const
{
    const int proto_area = mH * mW;

    // Linear combination: mask_logits = sum_m(coeff[m] * proto[m])
    cv::Mat logits(mH, mW, CV_32F, cv::Scalar(0.f));
    float *lptr = reinterpret_cast<float*>(logits.data);
    for (int m = 0; m < num_masks_; ++m)
    {
        const float c = coeff[m];
        const float *p = proto_data + m * proto_area;
        for (int px = 0; px < proto_area; ++px)
            lptr[px] += c * p[px];
    }

    // Sigmoid
    cv::exp(-logits, logits);          // reuse buffer
    logits = 1.f / (1.f + logits);    // sigmoid in-place via expression

    // Resize prototype mask from (mH, mW) → (640, 640)
    cv::Mat mask_fullsize;
    cv::resize(logits, mask_fullsize, cv::Size(model_input_w_, model_input_h_),
               0, 0, cv::INTER_LINEAR);

    // Crop to bbox region in 640x640 space, then back-project to orig size
    cv::Rect safe_bbox(
        std::clamp(bbox_lb.x, 0, static_cast<int>(model_input_w_) - 1),
        std::clamp(bbox_lb.y, 0, static_cast<int>(model_input_h_) - 1),
        std::clamp(bbox_lb.width,  1, static_cast<int>(model_input_w_) - bbox_lb.x),
        std::clamp(bbox_lb.height, 1, static_cast<int>(model_input_h_) - bbox_lb.y));

    // Build full-res binary mask in original image coordinates
    cv::Mat full_mask = cv::Mat::zeros(orig_h, orig_w, CV_8U);

    // Map each pixel in orig image back to letterboxed space and threshold
    for (int row = 0; row < orig_h; ++row)
    {
        for (int col = 0; col < orig_w; ++col)
        {
            float lb_x = col * scale + pad_w;
            float lb_y = row * scale + pad_h;
            int ix = static_cast<int>(std::round(lb_x));
            int iy = static_cast<int>(std::round(lb_y));
            if (ix < 0 || iy < 0 || ix >= model_input_w_ || iy >= model_input_h_)
                continue;
            if (mask_fullsize.at<float>(iy, ix) > 0.5f)
                full_mask.at<uint8_t>(row, col) = 255;
        }
    }

    // Mask outside bbox (keeps only the instance region)
    cv::Rect orig_bbox(
        static_cast<int>((safe_bbox.x - pad_w) / scale),
        static_cast<int>((safe_bbox.y - pad_h) / scale),
        static_cast<int>(safe_bbox.width  / scale),
        static_cast<int>(safe_bbox.height / scale));
    orig_bbox &= cv::Rect(0, 0, orig_w, orig_h);   // clamp to image

    cv::Mat bbox_mask = cv::Mat::zeros(orig_h, orig_w, CV_8U);
    full_mask(orig_bbox).copyTo(bbox_mask(orig_bbox));

    return bbox_mask;
}

// ─────────────────────────────────────────────────────────────────────────────
// Utilities
// ─────────────────────────────────────────────────────────────────────────────

cv::Point WeedDetector::maskCentroid(const cv::Mat &mask) const
{
    cv::Moments mu = cv::moments(mask, true);
    if (mu.m00 == 0.0)
        return {mask.cols / 2, mask.rows / 2}; // fallback: image centre
    return {static_cast<int>(mu.m10 / mu.m00),
            static_cast<int>(mu.m01 / mu.m00)};
}

double WeedDetector::getDepthMedian(const cv::Mat &depth, const cv::Point &center) const
{
    const int half = depth_roi_size_ / 2;
    std::vector<double> values;
    values.reserve((depth_roi_size_ + 1) * (depth_roi_size_ + 1));

    for (int dy = -half; dy <= half; ++dy)
    {
        for (int dx = -half; dx <= half; ++dx)
        {
            int x = center.x + dx;
            int y = center.y + dy;
            if (x < 0 || y < 0 || x >= depth.cols || y >= depth.rows)
                continue;

            double d = 0.0;
            // Support both 32F (metres, RealSense default) and 16U (millimetres)
            if (depth.type() == CV_32F)
                d = static_cast<double>(depth.at<float>(y, x));
            else if (depth.type() == CV_16U)
                d = static_cast<double>(depth.at<uint16_t>(y, x)) * 0.001;
            else
                d = depth.at<double>(y, x);

            if (std::isfinite(d) && d > 0.02)
                values.push_back(d);
        }
    }

    if (values.empty())
        return 0.0;

    std::nth_element(values.begin(),
                     values.begin() + values.size() / 2,
                     values.end());
    return values[values.size() / 2];
}

} // namespace arm_mazzolini
