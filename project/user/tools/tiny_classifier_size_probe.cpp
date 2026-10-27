#include <net.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <cstdio>
#include <limits>

namespace
{
constexpr int kClassCount = 3;
constexpr const char *kLabels[kClassCount] = {"supplies", "vehicle", "weapon"};
constexpr float kMean[kClassCount] = {123.675f, 116.28f, 103.53f};
constexpr float kNorm[kClassCount] = {0.01712475f, 0.017507f, 0.01742919f};

bool RunVariant(ncnn::Net &net, const cv::Mat &source_bgr, int side)
{
    cv::Mat resized_bgr;
    cv::resize(source_bgr, resized_bgr, cv::Size(side, side));
    cv::Mat resized_rgb;
    cv::cvtColor(resized_bgr, resized_rgb, cv::COLOR_BGR2RGB);
    ncnn::Mat input = ncnn::Mat::from_pixels(
        resized_rgb.data, ncnn::Mat::PIXEL_RGB, side, side);
    if (input.empty())
    {
        return false;
    }
    input.substract_mean_normalize(kMean, kNorm);

    ncnn::Extractor extractor = net.create_extractor();
    ncnn::Mat logits;
    if (extractor.input("in0", input) != 0 || extractor.extract("out0", logits) != 0 ||
        logits.empty() || logits.w != kClassCount)
    {
        return false;
    }

    float max_logit = -std::numeric_limits<float>::infinity();
    for (int index = 0; index < kClassCount; ++index)
    {
        if (!std::isfinite(logits[index]))
        {
            return false;
        }
        max_logit = std::max(max_logit, logits[index]);
    }
    float probabilities[kClassCount] = {};
    float sum = 0.0f;
    for (int index = 0; index < kClassCount; ++index)
    {
        probabilities[index] = std::exp(logits[index] - max_logit);
        sum += probabilities[index];
    }
    int top = 0;
    for (int index = 0; index < kClassCount; ++index)
    {
        probabilities[index] /= sum;
        if (probabilities[index] > probabilities[top])
        {
            top = index;
        }
    }
    std::printf("side=%d result=%s probabilities=%.9f,%.9f,%.9f\n",
                side, kLabels[top], probabilities[0], probabilities[1], probabilities[2]);
    return true;
}
}

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        std::fprintf(stderr, "usage: %s PARAM BIN ROI_BGR\n", argv[0]);
        return 2;
    }

    cv::Mat roi_bgr = cv::imread(argv[3], cv::IMREAD_COLOR);
    if (roi_bgr.empty())
    {
        std::fprintf(stderr, "cannot read ROI: %s\n", argv[3]);
        return 3;
    }
    ncnn::Net net;
    net.opt.num_threads = 1;
    net.opt.use_vulkan_compute = false;
    if (net.load_param(argv[1]) != 0 || net.load_model(argv[2]) != 0)
    {
        std::fprintf(stderr, "cannot load model\n");
        return 4;
    }

    return RunVariant(net, roi_bgr, 64) && RunVariant(net, roi_bgr, 96) ? 0 : 5;
}
