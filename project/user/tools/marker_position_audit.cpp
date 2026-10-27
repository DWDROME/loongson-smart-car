#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>

namespace
{
bool IsBaseImage(const std::filesystem::path &path)
{
    const std::string name = path.filename().string();
    const size_t first = name.find('_');
    const size_t dot = name.rfind(".jpg");
    return dot != std::string::npos && first != std::string::npos &&
           name.find('_', first + 1) == std::string::npos;
}

bool FindMarker(const std::filesystem::path &path, cv::Point2f &center)
{
    const cv::Mat image = cv::imread(path.string(), cv::IMREAD_COLOR);
    if (image.empty())
    {
        return false;
    }
    cv::Mat hsv;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::Mat low_hue;
    cv::Mat high_hue;
    cv::inRange(hsv, cv::Scalar(0, 60, 60), cv::Scalar(12, 255, 255), low_hue);
    cv::inRange(hsv, cv::Scalar(168, 60, 60), cv::Scalar(180, 255, 255), high_hue);
    cv::Mat mask = low_hue | high_hue;

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int count = cv::connectedComponentsWithStats(
        mask, labels, stats, centroids, 8, CV_32S, cv::CCL_GRANA);
    int first = -1;
    int second = -1;
    for (int index = 1; index < count; ++index)
    {
        if (stats.at<int>(index, cv::CC_STAT_AREA) < 3)
        {
            continue;
        }
        if (first < 0 || stats.at<int>(index, cv::CC_STAT_AREA) > stats.at<int>(first, cv::CC_STAT_AREA))
        {
            second = first;
            first = index;
        }
        else if (second < 0 || stats.at<int>(index, cv::CC_STAT_AREA) > stats.at<int>(second, cv::CC_STAT_AREA))
        {
            second = index;
        }
    }
    if (first < 0)
    {
        return false;
    }
    const int chosen =
        second >= 0 && centroids.at<double>(second, 1) > centroids.at<double>(first, 1)
            ? second
            : first;
    center = cv::Point2f(static_cast<float>(centroids.at<double>(chosen, 0)),
                         static_cast<float>(centroids.at<double>(chosen, 1)));
    return true;
}

float Median(std::vector<float> values)
{
    std::sort(values.begin(), values.end());
    const size_t middle = values.size() / 2;
    return values.size() % 2 == 0 ? (values[middle - 1] + values[middle]) * 0.5f : values[middle];
}
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::fprintf(stderr, "usage: %s CLASS_DIRECTORY\n", argv[0]);
        return 2;
    }

    std::vector<float> xs;
    std::vector<float> ys;
    int base_count = 0;
    for (const auto &entry : std::filesystem::directory_iterator(argv[1]))
    {
        if (!entry.is_regular_file() || !IsBaseImage(entry.path()))
        {
            continue;
        }
        ++base_count;
        cv::Point2f center;
        if (FindMarker(entry.path(), center))
        {
            xs.push_back(center.x);
            ys.push_back(center.y);
        }
    }

    if (xs.empty())
    {
        std::fprintf(stderr, "no marker found in %d base images\n", base_count);
        return 3;
    }
    std::printf("base_images=%d markers=%zu median_marker_xy=%.3f,%.3f\n",
                base_count, xs.size(), Median(xs), Median(ys));
    return 0;
}
