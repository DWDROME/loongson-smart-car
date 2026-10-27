#include "lq_ncnn.hpp"

#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <map>
#include <string>
#include <vector>

namespace
{
constexpr std::array<const char *, LQ_NCNN::kClassCount> kLabels = {
    "supplies",
    "vehicle",
    "weapon",
};

struct FineStats
{
    int expected_class = -1;
    long total = 0;
    long correct = 0;
    std::array<long, LQ_NCNN::kClassCount> predicted = {};
};

bool IsImageFile(const char *name)
{
    const char *extension = std::strrchr(name, '.');
    if (extension == nullptr)
    {
        return false;
    }
    return std::strcmp(extension, ".jpg") == 0 || std::strcmp(extension, ".JPG") == 0 ||
           std::strcmp(extension, ".jpeg") == 0 || std::strcmp(extension, ".JPEG") == 0 ||
           std::strcmp(extension, ".png") == 0 || std::strcmp(extension, ".PNG") == 0;
}

std::string FineLabel(const std::string &file_name)
{
    const std::string::size_type explicit_separator = file_name.find("__");
    if (explicit_separator != std::string::npos)
    {
        return file_name.substr(0, explicit_separator);
    }
    const std::string::size_type separator = file_name.find('_');
    return separator == std::string::npos ? file_name : file_name.substr(0, separator);
}

std::string CsvEscape(const std::string &value)
{
    bool needs_quotes = false;
    std::string escaped;
    escaped.reserve(value.size() + 2);
    for (const char character : value)
    {
        if (character == '"')
        {
            escaped += "\"\"";
            needs_quotes = true;
        }
        else
        {
            escaped += character;
            if (character == ',' || character == '\n' || character == '\r')
            {
                needs_quotes = true;
            }
        }
    }
    return needs_quotes ? '"' + escaped + '"' : escaped;
}

bool ParsePositiveInt(const char *text, int &value)
{
    if (text == nullptr || text[0] == '\0')
    {
        return false;
    }
    errno = 0;
    char *end = nullptr;
    const long parsed = std::strtol(text, &end, 10);
    if (errno != 0 || end == text || end == nullptr || end[0] != '\0' || parsed <= 0 ||
        parsed > 1000000L)
    {
        return false;
    }
    value = static_cast<int>(parsed);
    return true;
}

std::vector<std::string> ListImages(const std::string &directory)
{
    std::vector<std::string> images;
    DIR *dir = opendir(directory.c_str());
    if (dir == nullptr)
    {
        return images;
    }

    for (dirent *entry = readdir(dir); entry != nullptr; entry = readdir(dir))
    {
        if (IsImageFile(entry->d_name))
        {
            images.emplace_back(entry->d_name);
        }
    }
    closedir(dir);
    std::sort(images.begin(), images.end());
    return images;
}

void PrintUsage(const char *program)
{
    std::fprintf(stderr,
                 "usage: %s PARAM BIN DATASET_ROOT [MAX_IMAGES_PER_CLASS [RESULTS_CSV]]\n"
                 "  DATASET_ROOT must contain provider 64x64 images under supplies/, vehicle/, weapon/.\n",
                 program);
}
}

int main(int argc, char **argv)
{
    if (argc < 4 || argc > 6)
    {
        PrintUsage(argv[0]);
        return 2;
    }

    int max_images_per_class = -1;
    if (argc >= 5 && !ParsePositiveInt(argv[4], max_images_per_class))
    {
        std::fprintf(stderr, "invalid MAX_IMAGES_PER_CLASS: %s\n", argv[4]);
        return 2;
    }

    LQ_NCNN classifier;
    if (!classifier.Init(argv[1], argv[2]))
    {
        std::fprintf(stderr, "NCNN dataset eval init failed: %s\n", classifier.LastError());
        return 3;
    }

    FILE *results_csv = nullptr;
    if (argc == 6)
    {
        results_csv = std::fopen(argv[5], "w");
        if (results_csv == nullptr)
        {
            std::fprintf(stderr, "cannot open RESULTS_CSV: %s\n", argv[5]);
            return 4;
        }
        if (std::fprintf(results_csv,
                         "relative_path,fine_label,expected_label,predicted_label,"
                         "prob_supplies,prob_vehicle,prob_weapon,confidence,infer_us,error\n") < 0)
        {
            std::fprintf(stderr, "cannot write RESULTS_CSV header: %s\n", argv[5]);
            std::fclose(results_csv);
            return 4;
        }
    }

    const std::string dataset_root(argv[3]);
    std::array<std::array<long, LQ_NCNN::kClassCount>, LQ_NCNN::kClassCount> confusion = {};
    std::array<long, LQ_NCNN::kClassCount> skipped = {};
    std::map<std::string, FineStats> fine_stats;
    long processed = 0;
    long total_infer_us = 0;

    for (int expected = 0; expected < LQ_NCNN::kClassCount; ++expected)
    {
        const std::string class_dir = dataset_root + "/" + kLabels[expected];
        std::vector<std::string> images = ListImages(class_dir);
        if (images.empty())
        {
            std::fprintf(stderr, "no image files found: %s\n", class_dir.c_str());
            if (results_csv != nullptr)
            {
                std::fclose(results_csv);
            }
            return 5;
        }
        if (max_images_per_class > 0 && static_cast<int>(images.size()) > max_images_per_class)
        {
            images.resize(static_cast<size_t>(max_images_per_class));
        }

        for (const std::string &file_name : images)
        {
            const cv::Mat roi_bgr = cv::imread(class_dir + "/" + file_name, cv::IMREAD_COLOR);
            const std::string relative_path = std::string(kLabels[expected]) + "/" + file_name;
            const std::string fine_label = FineLabel(file_name);
            int predicted = -1;
            float confidence = 0.0f;
            long infer_us = 0;
            float probabilities[LQ_NCNN::kClassCount] = {};
            if (!classifier.Infer(roi_bgr, predicted, confidence, infer_us, probabilities))
            {
                const std::string error = classifier.LastError();
                if (skipped[expected] < 5)
                {
                    std::fprintf(stderr, "skip %s/%s: %s\n", kLabels[expected],
                                 file_name.c_str(), error.c_str());
                }
                if (results_csv != nullptr &&
                    std::fprintf(results_csv, "%s,%s,%s,,,,,,,%s\n",
                                 CsvEscape(relative_path).c_str(), CsvEscape(fine_label).c_str(),
                                 kLabels[expected], CsvEscape(error).c_str()) < 0)
                {
                    std::fprintf(stderr, "cannot write RESULTS_CSV failure row\n");
                    std::fclose(results_csv);
                    return 6;
                }
                ++skipped[expected];
                continue;
            }

            if (results_csv != nullptr &&
                std::fprintf(results_csv, "%s,%s,%s,%s,%.9f,%.9f,%.9f,%.9f,%ld,\n",
                             CsvEscape(relative_path).c_str(), CsvEscape(fine_label).c_str(),
                             kLabels[expected], kLabels[predicted], probabilities[0],
                             probabilities[1], probabilities[2], confidence, infer_us) < 0)
            {
                std::fprintf(stderr, "cannot write RESULTS_CSV result row\n");
                std::fclose(results_csv);
                return 6;
            }

            ++confusion[expected][predicted];
            ++processed;
            total_infer_us += infer_us;
            const std::string fine_key = std::string(kLabels[expected]) + "/" + fine_label;
            FineStats &stats = fine_stats[fine_key];
            stats.expected_class = expected;
            ++stats.total;
            ++stats.predicted[predicted];
            if (predicted == expected)
            {
                ++stats.correct;
            }

            if ((processed % 1000) == 0)
            {
                std::printf("progress processed=%ld last=%s/%s\n", processed,
                            kLabels[expected], file_name.c_str());
                std::fflush(stdout);
            }
        }
    }

    if (processed == 0)
    {
        std::fprintf(stderr, "no valid dataset images were evaluated\n");
        if (results_csv != nullptr)
        {
            std::fclose(results_csv);
        }
        return 7;
    }

    long correct = 0;
    std::array<long, LQ_NCNN::kClassCount> actual_totals = {};
    std::array<long, LQ_NCNN::kClassCount> predicted_totals = {};
    for (int actual = 0; actual < LQ_NCNN::kClassCount; ++actual)
    {
        for (int predicted = 0; predicted < LQ_NCNN::kClassCount; ++predicted)
        {
            actual_totals[actual] += confusion[actual][predicted];
            predicted_totals[predicted] += confusion[actual][predicted];
        }
        correct += confusion[actual][actual];
    }

    std::printf("dataset_eval model=%s processed=%ld correct=%ld accuracy=%.6f avg_infer_us=%.2f\n",
                LQ_NCNN::ModelId(), processed, correct,
                static_cast<double>(correct) / static_cast<double>(processed),
                static_cast<double>(total_infer_us) / static_cast<double>(processed));
    std::printf("labels=supplies,vehicle,weapon\n");
    std::printf("confusion_rows_actual_cols_predicted\n");
    for (int actual = 0; actual < LQ_NCNN::kClassCount; ++actual)
    {
        std::printf("%s,%ld,%ld,%ld\n", kLabels[actual], confusion[actual][0],
                    confusion[actual][1], confusion[actual][2]);
    }
    for (int label = 0; label < LQ_NCNN::kClassCount; ++label)
    {
        const double precision = predicted_totals[label] == 0
                                     ? 0.0
                                     : static_cast<double>(confusion[label][label]) /
                                           static_cast<double>(predicted_totals[label]);
        const double recall = actual_totals[label] == 0
                                  ? 0.0
                                  : static_cast<double>(confusion[label][label]) /
                                        static_cast<double>(actual_totals[label]);
        std::printf("class=%s count=%ld precision=%.6f recall=%.6f skipped=%ld\n",
                    kLabels[label], actual_totals[label], precision, recall, skipped[label]);
    }
    std::printf("fine_classes\n");
    for (const auto &entry : fine_stats)
    {
        const FineStats &stats = entry.second;
        std::printf("fine=%s expected=%s count=%ld accuracy=%.6f predicted=%ld,%ld,%ld\n",
                    entry.first.c_str(), kLabels[stats.expected_class], stats.total,
                    static_cast<double>(stats.correct) / static_cast<double>(stats.total),
                    stats.predicted[0], stats.predicted[1], stats.predicted[2]);
    }
    if (results_csv != nullptr && std::fclose(results_csv) != 0)
    {
        std::fprintf(stderr, "cannot finalize RESULTS_CSV: %s\n", argv[5]);
        return 6;
    }
    const long skipped_total = skipped[0] + skipped[1] + skipped[2];
    if (skipped_total != 0)
    {
        std::fprintf(stderr, "dataset evaluation incomplete: skipped=%ld\n", skipped_total);
        return 8;
    }
    return 0;
}
