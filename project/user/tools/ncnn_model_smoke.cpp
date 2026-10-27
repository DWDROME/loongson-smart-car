#include "lq_ncnn.hpp"

#include <opencv2/imgcodecs.hpp>

#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace
{
constexpr float kSelfTestExpected[LQ_NCNN::kClassCount] = {
    0.383568525f,
    0.534451544f,
    0.081979938f,
};
constexpr float kSelfTestTolerance = 1.0e-4f;

cv::Mat BuildSelfTestRoi()
{
    // RGB=(176,160,176) gives a deliberately non-saturated three-class response.
    return cv::Mat(LQ_NCNN::kInputHeight, LQ_NCNN::kInputWidth,
                   CV_8UC3, cv::Scalar(176, 160, 176)).clone();
}

bool ParseFloat(const char *text, float &value)
{
    if (text == nullptr || text[0] == '\0')
    {
        return false;
    }
    errno = 0;
    char *end = nullptr;
    value = std::strtof(text, &end);
    return errno == 0 && end != text && end != nullptr && end[0] == '\0' &&
           std::isfinite(value);
}

void PrintUsage(const char *program)
{
    std::fprintf(stderr,
                 "usage: %s PARAM BIN MODEL_INPUT_BGR_64X64 "
                 "[EXPECTED_SUPPLIES EXPECTED_VEHICLE EXPECTED_WEAPON [TOLERANCE]]\n"
                 "       %s PARAM BIN --self-test\n",
                 program,
                 program);
}
}

int main(int argc, char **argv)
{
    if (argc != 4 && argc != 7 && argc != 8)
    {
        PrintUsage(argv[0]);
        return 2;
    }

    LQ_NCNN classifier;
    if (!classifier.Init(argv[1], argv[2]))
    {
        std::fprintf(stderr, "NCNN smoke init failed: %s\n", classifier.LastError());
        return 3;
    }

    const bool self_test = std::strcmp(argv[3], "--self-test") == 0;
    if (self_test && argc != 4)
    {
        PrintUsage(argv[0]);
        return 2;
    }
    const cv::Mat roi_bgr = self_test
                                ? BuildSelfTestRoi()
                                : cv::imread(argv[3], cv::IMREAD_COLOR);
    int class_idx = -1;
    float confidence = 0.0f;
    long infer_us = 0;
    float probabilities[LQ_NCNN::kClassCount] = {};
    if (!classifier.Infer(roi_bgr, class_idx, confidence, infer_us, probabilities))
    {
        std::fprintf(stderr, "NCNN smoke inference failed: %s\n", classifier.LastError());
        return 4;
    }

    std::printf("model=%s result=%s class_idx=%d confidence=%.9f infer_us=%ld\n",
                LQ_NCNN::ModelId(), classifier.Label(class_idx),
                class_idx, confidence, infer_us);
    std::printf("probabilities=%.9f,%.9f,%.9f order=supplies,vehicle,weapon\n",
                probabilities[0], probabilities[1], probabilities[2]);

    if (argc == 4 && !self_test)
    {
        return 0;
    }

    float expected[LQ_NCNN::kClassCount] = {};
    float tolerance = kSelfTestTolerance;
    if (self_test)
    {
        for (int i = 0; i < LQ_NCNN::kClassCount; ++i)
        {
            expected[i] = kSelfTestExpected[i];
        }
        std::printf("self_test_reference=current_camera_own6_tiny64_w1_v2 expected=%.9f,%.9f,%.9f\n",
                    expected[0], expected[1], expected[2]);
    }
    else
    {
        for (int i = 0; i < LQ_NCNN::kClassCount; ++i)
        {
            if (!ParseFloat(argv[4 + i], expected[i]))
            {
                std::fprintf(stderr, "invalid expected probability: %s\n", argv[4 + i]);
                return 2;
            }
        }
        if (argc == 8 && (!ParseFloat(argv[7], tolerance) || tolerance < 0.0f))
        {
            std::fprintf(stderr, "invalid tolerance: %s\n", argv[7]);
            return 2;
        }
    }

    float max_absolute_difference = 0.0f;
    for (int i = 0; i < LQ_NCNN::kClassCount; ++i)
    {
        const float difference = std::fabs(probabilities[i] - expected[i]);
        if (difference > max_absolute_difference)
        {
            max_absolute_difference = difference;
        }
    }

    std::printf("parity_max_abs=%.9g tolerance=%.9g status=%s\n",
                max_absolute_difference, tolerance,
                max_absolute_difference <= tolerance ? "passed" : "failed");
    return max_absolute_difference <= tolerance ? 0 : 5;
}
