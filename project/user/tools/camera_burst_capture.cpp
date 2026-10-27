#include "lq_camera_ex.hpp"
#include "zf_device_uvc.hpp"

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <vector>

namespace
{
enum class CaptureFormat
{
    kMjpg,
    kYuyv,
};

enum class CameraBackend
{
    kLq,
    kZf,
};

struct Options
{
    std::string output_dir;
    int count = 200;
    double hz = 10.0;
    int exposure = 90;
    int jpeg_quality = 95;
    int warmup_frames = 10;
    bool save_images = true;
    bool save_png = false;
    bool auto_exposure = false;
    CaptureFormat format = CaptureFormat::kMjpg;
    CameraBackend backend = CameraBackend::kLq;
};

void PrintUsage(const char *program)
{
    std::fprintf(stderr,
                 "usage: %s --output DIR [--count 200] [--hz 10] [--exposure 90] "
                 "[--quality 95] [--warmup 10] [--backend lq|zf] "
                 "[--format mjpg|yuyv] [--auto-exposure] [--png] [--no-save]\n",
                 program);
}

bool ParseInt(const char *text, int minimum, int maximum, int &value)
{
    if (text == nullptr || text[0] == '\0')
    {
        return false;
    }
    errno = 0;
    char *end = nullptr;
    const long parsed = std::strtol(text, &end, 10);
    if (errno != 0 || end == text || end == nullptr || end[0] != '\0' ||
        parsed < minimum || parsed > maximum)
    {
        return false;
    }
    value = static_cast<int>(parsed);
    return true;
}

bool ParseDouble(const char *text, double minimum, double maximum, double &value)
{
    if (text == nullptr || text[0] == '\0')
    {
        return false;
    }
    errno = 0;
    char *end = nullptr;
    const double parsed = std::strtod(text, &end);
    if (errno != 0 || end == text || end == nullptr || end[0] != '\0' ||
        parsed < minimum || parsed > maximum)
    {
        return false;
    }
    value = parsed;
    return true;
}

bool ParseOptions(int argc, char **argv, Options &options)
{
    for (int index = 1; index < argc; ++index)
    {
        const char *arg = argv[index];
        if (std::strcmp(arg, "--no-save") == 0)
        {
            options.save_images = false;
        }
        else if (std::strcmp(arg, "--png") == 0)
        {
            options.save_png = true;
        }
        else if (std::strcmp(arg, "--auto-exposure") == 0)
        {
            options.auto_exposure = true;
        }
        else if (std::strcmp(arg, "--backend") == 0 && index + 1 < argc)
        {
            const char *backend = argv[++index];
            if (std::strcmp(backend, "lq") == 0)
            {
                options.backend = CameraBackend::kLq;
            }
            else if (std::strcmp(backend, "zf") == 0)
            {
                options.backend = CameraBackend::kZf;
            }
            else
            {
                return false;
            }
        }
        else if (std::strcmp(arg, "--format") == 0 && index + 1 < argc)
        {
            const char *format = argv[++index];
            if (std::strcmp(format, "mjpg") == 0)
            {
                options.format = CaptureFormat::kMjpg;
            }
            else if (std::strcmp(format, "yuyv") == 0)
            {
                options.format = CaptureFormat::kYuyv;
            }
            else
            {
                return false;
            }
        }
        else if (std::strcmp(arg, "--output") == 0 && index + 1 < argc)
        {
            options.output_dir = argv[++index];
        }
        else if (std::strcmp(arg, "--count") == 0 && index + 1 < argc)
        {
            if (!ParseInt(argv[++index], 1, 100000, options.count))
            {
                return false;
            }
        }
        else if (std::strcmp(arg, "--hz") == 0 && index + 1 < argc)
        {
            if (!ParseDouble(argv[++index], 0.1, 120.0, options.hz))
            {
                return false;
            }
        }
        else if (std::strcmp(arg, "--exposure") == 0 && index + 1 < argc)
        {
            if (!ParseInt(argv[++index], 1, 2500, options.exposure))
            {
                return false;
            }
        }
        else if (std::strcmp(arg, "--quality") == 0 && index + 1 < argc)
        {
            if (!ParseInt(argv[++index], 1, 100, options.jpeg_quality))
            {
                return false;
            }
        }
        else if (std::strcmp(arg, "--warmup") == 0 && index + 1 < argc)
        {
            if (!ParseInt(argv[++index], 0, 1000, options.warmup_frames))
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }
    return !options.save_images || !options.output_dir.empty();
}

double Milliseconds(std::chrono::steady_clock::duration duration)
{
    return std::chrono::duration<double, std::milli>(duration).count();
}
}

int main(int argc, char **argv)
{
    Options options;
    if (!ParseOptions(argc, argv, options))
    {
        PrintUsage(argv[0]);
        return 2;
    }

    FILE *csv = nullptr;
    if (options.save_images)
    {
        if (mkdir(options.output_dir.c_str(), 0755) != 0 && errno != EEXIST)
        {
            std::fprintf(stderr, "cannot create output directory %s: %s\n",
                         options.output_dir.c_str(), std::strerror(errno));
            return 3;
        }
        const std::string csv_path = options.output_dir + "/capture.csv";
        csv = std::fopen(csv_path.c_str(), "w");
        if (csv == nullptr)
        {
            std::fprintf(stderr, "cannot create %s: %s\n",
                         csv_path.c_str(), std::strerror(errno));
            return 3;
        }
        std::fprintf(csv,
                     "attempt,frame,scheduled_ms,start_ms,capture_ms,save_ms,late_ms,"
                     "y_mean,laplacian_var,path\n");
    }

    const bool use_zf = options.backend == CameraBackend::kZf;
    const bool use_mjpg = options.format == CaptureFormat::kMjpg;
    std::unique_ptr<lq_camera_ex> lq_camera;
    std::unique_ptr<zf_device_uvc> zf_camera;
    if (use_zf)
    {
        zf_camera = std::make_unique<zf_device_uvc>();
        if (zf_camera->init(UVC_PATH) != 0 || !zf_camera->is_camera_opened())
        {
            std::fprintf(stderr, "ZF camera open failed\n");
            if (csv != nullptr)
            {
                std::fclose(csv);
            }
            return 4;
        }
    }
    else
    {
        const uint16_t requested_fps = use_mjpg ? 120 : 60;
        // In the bundled LQ archive, 0CPU_MJPG selects the YUYV capture path.
        const lq_camera_format_t camera_format = use_mjpg ? LQ_CAMERA_HIGH_MJPG
                                                           : LQ_CAMERA_0CPU_MJPG;
        lq_camera = std::make_unique<lq_camera_ex>(320, 240, requested_fps,
                                                   camera_format, LQ_CAMERA_PATH);
    }
    if (!use_zf && !lq_camera->is_cam_opened())
    {
        std::fprintf(stderr, "LQ camera open failed\n");
        if (csv != nullptr)
        {
            std::fclose(csv);
        }
        return 4;
    }

    bool exposure_ok = true;
    if (use_zf)
    {
        if (!options.auto_exposure)
        {
            exposure_ok = zf_camera->set_auto_exposure(UVC_AUTO_EXPOSURE_DISABLE) == 0 &&
                          zf_camera->set_exposure_value(options.exposure) == 0;
        }
    }
    else if (options.auto_exposure)
    {
        std::fprintf(stderr, "--auto-exposure is only supported by the ZF backend\n");
        exposure_ok = false;
    }
    else
    {
        exposure_ok = lq_camera->set_exposure_manual(
            static_cast<int16_t>(options.exposure));
    }
    if (!exposure_ok)
    {
        std::fprintf(stderr, "exposure configuration rejected\n");
        if (csv != nullptr)
        {
            std::fclose(csv);
        }
        return 5;
    }

    if (use_zf)
    {
        std::printf("camera_mode backend=ZF width=%d height=%d fps=%d format=YUYV "
                    "exposure=%s",
                    UVC_WIDTH, UVC_HEIGHT, UVC_FPS,
                    options.auto_exposure ? "auto" : "manual");
        if (!options.auto_exposure)
        {
            std::printf(":%d", options.exposure);
        }
        std::printf(" actual_auto=%.3f actual_exposure=%.3f\n",
                    zf_camera->get_auto_exposure_mode(),
                    zf_camera->get_current_exposure());
    }
    else
    {
        const char *format_name = use_mjpg ? "MJPG" : "YUYV";
        std::printf("camera_mode backend=LQ width=%u height=%u fps=%u format=%s "
                    "exposure=manual:%d\n",
                    lq_camera->get_camera_width(), lq_camera->get_camera_height(),
                    lq_camera->get_camera_fps(), format_name, options.exposure);
    }
    std::printf("capture_request count=%d hz=%.3f save=%d encoding=%s quality=%d "
                "warmup=%d output=%s\n",
                options.count, options.hz, options.save_images ? 1 : 0,
                options.save_png ? "PNG" : "JPEG", options.jpeg_quality,
                options.warmup_frames,
                options.save_images ? options.output_dir.c_str() : "none");

    for (int index = 0; index < options.warmup_frames; ++index)
    {
        cv::Mat warmup_frame;
        if (use_zf)
        {
            if (zf_camera->wait_image_refresh() == 0)
            {
                warmup_frame = zf_camera->get_frame_bgr();
            }
        }
        else
        {
            warmup_frame = lq_camera->get_frame_raw();
        }
        if (warmup_frame.empty())
        {
            std::fprintf(stderr, "warmup frame failed at %d\n", index);
        }
    }

    const auto period = std::chrono::duration<double>(1.0 / options.hz);
    const auto capture_start = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    int attempts = 0;
    int saved = 0;
    int empty_frames = 0;
    int late_frames = 0;
    double capture_ms_sum = 0.0;
    double save_ms_sum = 0.0;
    double y_mean_sum = 0.0;
    double laplacian_var_sum = 0.0;
    double max_late_ms = 0.0;
    const std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, options.jpeg_quality};
    const std::vector<int> png_params = {cv::IMWRITE_PNG_COMPRESSION, 1};

    while (saved < options.count && attempts < options.count * 3)
    {
        const auto deadline = capture_start + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                  period * static_cast<double>(attempts));
        std::this_thread::sleep_until(deadline);
        const auto frame_start = std::chrono::steady_clock::now();
        const double late_ms = Milliseconds(frame_start - deadline);
        if (late_ms > 1.0)
        {
            ++late_frames;
        }
        if (late_ms > max_late_ms)
        {
            max_late_ms = late_ms;
        }

        cv::Mat frame;
        if (use_zf)
        {
            if (zf_camera->wait_image_refresh() == 0)
            {
                frame = zf_camera->get_frame_bgr();
            }
        }
        else
        {
            frame = lq_camera->get_frame_raw();
        }
        const auto frame_end = std::chrono::steady_clock::now();
        const double capture_ms = Milliseconds(frame_end - frame_start);
        capture_ms_sum += capture_ms;
        if (frame.empty())
        {
            ++empty_frames;
            ++attempts;
            continue;
        }

        const cv::Scalar bgr_mean = cv::mean(frame);
        const double y_mean = 0.114 * bgr_mean[0] + 0.587 * bgr_mean[1] +
                              0.299 * bgr_mean[2];
        cv::Mat gray;
        cv::Mat laplacian;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::Laplacian(gray, laplacian, CV_64F);
        cv::Scalar laplacian_mean;
        cv::Scalar laplacian_stddev;
        cv::meanStdDev(laplacian, laplacian_mean, laplacian_stddev);
        const double laplacian_var = laplacian_stddev[0] * laplacian_stddev[0];
        y_mean_sum += y_mean;
        laplacian_var_sum += laplacian_var;

        std::string path;
        double save_ms = 0.0;
        if (options.save_images)
        {
            char file_name[64] = {};
            std::snprintf(file_name, sizeof(file_name), "frame_%06d.%s", saved,
                          options.save_png ? "png" : "jpg");
            path = options.output_dir + "/" + file_name;
            const auto save_start = std::chrono::steady_clock::now();
            const bool write_ok = cv::imwrite(path, frame,
                                              options.save_png ? png_params : jpeg_params);
            const auto save_end = std::chrono::steady_clock::now();
            save_ms = Milliseconds(save_end - save_start);
            save_ms_sum += save_ms;
            if (!write_ok)
            {
                std::fprintf(stderr, "image write failed: %s\n", path.c_str());
                if (csv != nullptr)
                {
                    std::fclose(csv);
                }
                return 6;
            }
        }

        const double scheduled_ms = Milliseconds(deadline - capture_start);
        const double start_ms = Milliseconds(frame_start - capture_start);
        if (csv != nullptr)
        {
            std::fprintf(csv, "%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%s\n",
                         attempts, saved, scheduled_ms, start_ms, capture_ms,
                         save_ms, late_ms, y_mean, laplacian_var, path.c_str());
        }
        ++saved;
        ++attempts;
        if ((saved % 25) == 0 || saved == options.count)
        {
            std::printf("progress saved=%d/%d attempts=%d\n", saved, options.count, attempts);
        }
    }

    const auto capture_end = std::chrono::steady_clock::now();
    const double elapsed_s = std::chrono::duration<double>(capture_end - capture_start).count();
    if (csv != nullptr)
    {
        std::fclose(csv);
    }
    if (lq_camera)
    {
        lq_camera->stop_collect();
    }

    const double achieved_fps = elapsed_s > 0.0 ? static_cast<double>(saved) / elapsed_s : 0.0;
    std::printf("capture_summary requested=%d saved=%d attempts=%d empty=%d "
                "requested_hz=%.3f achieved_fps=%.3f elapsed_s=%.3f late_frames=%d "
                "max_late_ms=%.3f avg_capture_ms=%.3f avg_save_ms=%.3f "
                "avg_y=%.3f avg_laplacian_var=%.3f\n",
                options.count, saved, attempts, empty_frames, options.hz,
                achieved_fps, elapsed_s, late_frames, max_late_ms,
                attempts > 0 ? capture_ms_sum / attempts : 0.0,
                saved > 0 ? save_ms_sum / saved : 0.0,
                saved > 0 ? y_mean_sum / saved : 0.0,
                saved > 0 ? laplacian_var_sum / saved : 0.0);
    return saved == options.count ? 0 : 7;
}
