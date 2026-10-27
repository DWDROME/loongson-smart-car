#include "red_brick_filter.hpp"

#include <cstdlib>

namespace red_brick_filter
{
bool CalcWhiteRatio(const cv::Mat &roi_bgr, float &white_ratio)
{
    white_ratio = 0.0f;
    if (roi_bgr.empty() || roi_bgr.type() != CV_8UC3 ||
        roi_bgr.rows <= 0 || roi_bgr.cols <= 0)
    {
        return false;
    }

    size_t white_pixels = 0;
    for (int y = 0; y < roi_bgr.rows; ++y)
    {
        const cv::Vec3b *row = roi_bgr.ptr<cv::Vec3b>(y);
        for (int x = 0; x < roi_bgr.cols; ++x)
        {
            const int b = row[x][0];
            const int g = row[x][1];
            const int r = row[x][2];
            if (std::abs(r - g) < kWhitePixelDiffMax &&
                std::abs(r - b) < kWhitePixelDiffMax &&
                r > kWhitePixelBrightMin)
            {
                ++white_pixels;
            }
        }
    }

    white_ratio = static_cast<float>(white_pixels) /
                  static_cast<float>(roi_bgr.total());
    return true;
}
}
