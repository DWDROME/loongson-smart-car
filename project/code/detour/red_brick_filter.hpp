#pragma once

#include "red_event_types.hpp"

#include <opencv2/core.hpp>

namespace red_brick_filter
{
inline constexpr int kWhitePixelDiffMax = 25;
inline constexpr int kWhitePixelBrightMin = 120;

// Provider diagnostic: white pixels have similar RGB channels and sufficient brightness.
// This function only measures the ROI; it does not classify, log, or update vehicle state.
bool CalcWhiteRatio(const cv::Mat &roi_bgr, float &white_ratio);
}
