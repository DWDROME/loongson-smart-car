#pragma once

#include <cstdint>
#include <string>

#include <opencv2/core.hpp>

namespace dataset_capture
{

struct FrameSample
{
    cv::Mat frame_bgr;
    cv::Mat roi80_bgr;
    cv::Mat model_input64_bgr;
    cv::Point red_center;
    cv::Rect roi_rect;
    std::uint32_t frame_id = 0;
    float distance_cm = 0.0f;
    bool white_ratio_valid = false;
    float white_ratio = 0.0f;
    int class_idx = -1;
    float confidence = 0.0f;
    long infer_us = 0;
    float class_probs[3] = {0.0f, 0.0f, 0.0f};
    int exposure_config = -1;
    int exposure_mode = -1;
    int exposure_value = -1;
};

std::string LabelsJson();
std::string StatusJson();
bool IsActive();

// Returns the HTTP status code and writes a JSON response body.
int Start(const std::string &fine_label, const std::string &mode, std::string &response_json);
int Cancel(std::string &response_json);

// Called synchronously by the image thread only after ROI preparation and inference succeed.
void OnValidFrame(const FrameSample &sample);

} // namespace dataset_capture
