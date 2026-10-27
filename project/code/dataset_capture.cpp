#include "dataset_capture.hpp"

#include "lq_ncnn.hpp"
#include "run_mode.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

namespace dataset_capture
{
namespace
{

constexpr const char *kCaptureRoot = "/root/captures/web_dataset";
constexpr int kBurstHz = 5;
constexpr int kBurstDurationMs = 5000;
constexpr int kBurstPeriodMs = 1000 / kBurstHz;
constexpr int kBurstTarget = kBurstHz * kBurstDurationMs / 1000;
constexpr int kSingleTarget = 1;

struct LabelDefinition
{
    const char *fine;
    const char *business;
    const char *display;
};

constexpr LabelDefinition kLabels[] = {
    {"telescope", "supplies", "望远镜"},
    {"firstaid", "supplies", "急救包"},
    {"board_supplies", "supplies", "物资牌"},
    {"ambulance", "vehicle", "救护车"},
    {"armored", "vehicle", "装甲车"},
    {"gun", "weapon", "枪械"},
    {"explosive", "weapon", "爆炸物/手雷"},
    {"board_weapon", "weapon", "武器牌"},
};

enum class Phase
{
    kIdle,
    kCapturing,
    kCompleted,
    kCancelled,
    kError,
};

struct CaptureState
{
    std::mutex mutex;
    Phase phase = Phase::kIdle;
    bool active = false;
    bool writing = false;
    std::uint64_t generation = 0;
    std::uint32_t session_sequence = 0;
    std::string mode;
    std::string fine_label;
    std::string business_label;
    std::string display_label;
    std::string session_id;
    std::string session_path;
    std::string last_message;
    int target = 0;
    int scheduled = 0;
    int saved = 0;
    int skipped = 0;
    int errors = 0;
    std::chrono::steady_clock::time_point started;
    std::chrono::steady_clock::time_point next_due;
    std::chrono::steady_clock::time_point deadline;
};

struct WriteReservation
{
    std::uint64_t generation = 0;
    int slot = 0;
    std::string session_id;
    std::string session_path;
    std::string mode;
    std::string fine_label;
    std::string business_label;
};

CaptureState &State()
{
    static CaptureState state;
    return state;
}

const LabelDefinition *FindLabel(const std::string &fine_label)
{
    for (const LabelDefinition &label : kLabels)
    {
        if (fine_label == label.fine)
            return &label;
    }
    return nullptr;
}

const char *PhaseName(Phase phase)
{
    switch (phase)
    {
    case Phase::kIdle:
        return "idle";
    case Phase::kCapturing:
        return "capturing";
    case Phase::kCompleted:
        return "completed";
    case Phase::kCancelled:
        return "cancelled";
    case Phase::kError:
        return "error";
    }
    return "error";
}

std::uint64_t EpochMs()
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
}

std::string JsonEscape(const std::string &value)
{
    std::ostringstream escaped;
    for (unsigned char character : value)
    {
        switch (character)
        {
        case '\\':
            escaped << "\\\\";
            break;
        case '"':
            escaped << "\\\"";
            break;
        case '\n':
            escaped << "\\n";
            break;
        case '\r':
            escaped << "\\r";
            break;
        case '\t':
            escaped << "\\t";
            break;
        default:
            if (character < 0x20)
            {
                escaped << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                        << static_cast<int>(character) << std::dec;
            }
            else
            {
                escaped << static_cast<char>(character);
            }
        }
    }
    return escaped.str();
}

bool EnsureDirectory(const std::string &path, std::string &error)
{
    if (path.empty() || path[0] != '/')
    {
        error = "capture directory must be absolute";
        return false;
    }
    std::string current;
    std::size_t position = 1;
    while (position <= path.size())
    {
        const std::size_t slash = path.find('/', position);
        const std::string component = path.substr(
            position, slash == std::string::npos ? std::string::npos : slash - position);
        if (!component.empty())
        {
            current += "/" + component;
            if (mkdir(current.c_str(), 0755) != 0 && errno != EEXIST)
            {
                error = "mkdir failed for " + current + ": " + std::strerror(errno);
                return false;
            }
        }
        if (slash == std::string::npos)
            break;
        position = slash + 1;
    }
    return true;
}

std::string NewSessionId(CaptureState &state);

bool CreateSessionDirectory(CaptureState &state,
                            const LabelDefinition &label,
                            std::string &error)
{
    const std::string parent = std::string(kCaptureRoot) + "/" + label.business +
                               "/" + label.fine;
    if (!EnsureDirectory(parent, error))
        return false;

    for (int attempt = 0; attempt < 100; ++attempt)
    {
        state.session_id = NewSessionId(state);
        state.session_path = parent + "/" + state.session_id;
        if (mkdir(state.session_path.c_str(), 0755) == 0)
            return true;
        if (errno != EEXIST)
        {
            error = "mkdir failed for " + state.session_path + ": " +
                    std::strerror(errno);
            return false;
        }
    }

    error = "failed to allocate a unique capture session directory";
    return false;
}

bool WriteTextFile(const std::string &path, const std::string &content, std::string &error)
{
    FILE *file = std::fopen(path.c_str(), "w");
    if (file == nullptr)
    {
        error = "open failed for " + path + ": " + std::strerror(errno);
        return false;
    }
    const std::size_t written = std::fwrite(content.data(), 1, content.size(), file);
    const bool closed = std::fclose(file) == 0;
    if (written != content.size() || !closed)
    {
        error = "write failed for " + path;
        return false;
    }
    return true;
}

std::string NewSessionId(CaptureState &state)
{
    const std::time_t now = std::time(nullptr);
    struct tm local_time;
    localtime_r(&now, &local_time);
    char timestamp[32] = {};
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", &local_time);
    char session_id[64] = {};
    std::snprintf(session_id, sizeof(session_id), "%s_%d_%03u",
                  timestamp, static_cast<int>(getpid()), state.session_sequence++);
    return session_id;
}

void ExpireLocked(CaptureState &state, std::chrono::steady_clock::time_point now)
{
    if (!state.active || state.writing || now < state.deadline)
        return;
    const int remaining = std::max(0, state.target - state.scheduled);
    state.scheduled += remaining;
    state.skipped += remaining;
    state.active = false;
    state.phase = Phase::kCompleted;
    state.last_message = remaining == 0
                             ? "capture completed"
                             : "capture completed with invalid or missing ROI slots";
}

std::string BuildStatusLocked(CaptureState &state,
                              std::chrono::steady_clock::time_point now)
{
    ExpireLocked(state, now);
    long remaining_ms = 0;
    if (state.active)
    {
        remaining_ms = std::max<long>(
            0,
            std::chrono::duration_cast<std::chrono::milliseconds>(state.deadline - now)
                .count());
    }
    std::ostringstream json;
    json << "{\"phase\":\"" << PhaseName(state.phase)
         << "\",\"active\":" << (state.active ? "true" : "false")
         << ",\"writing\":" << (state.writing ? "true" : "false")
         << ",\"mode\":\"" << JsonEscape(state.mode)
         << "\",\"fineClass\":\"" << JsonEscape(state.fine_label)
         << "\",\"businessClass\":\"" << JsonEscape(state.business_label)
         << "\",\"displayLabel\":\"" << JsonEscape(state.display_label)
         << "\",\"sessionId\":\"" << JsonEscape(state.session_id)
         << "\",\"sessionPath\":\"" << JsonEscape(state.session_path)
         << "\",\"target\":" << state.target
         << ",\"scheduled\":" << state.scheduled
         << ",\"saved\":" << state.saved
         << ",\"skipped\":" << state.skipped
         << ",\"errors\":" << state.errors
         << ",\"remainingMs\":" << remaining_ms
         << ",\"frequencyHz\":" << kBurstHz
         << ",\"durationMs\":" << kBurstDurationMs
         << ",\"lastMessage\":\"" << JsonEscape(state.last_message) << "\"}";
    return json.str();
}

const char *ExposureOwner(int exposure_config)
{
    if (exposure_config < 0)
        return "driver-default";
    if (exposure_config == 0)
        return "app-auto";
    return "app-manual";
}

bool RenameFile(const std::string &temporary_path,
                const std::string &final_path,
                std::string &error)
{
    if (std::rename(temporary_path.c_str(), final_path.c_str()) != 0)
    {
        error = "rename failed for " + final_path + ": " + std::strerror(errno);
        return false;
    }
    return true;
}

void RemovePaths(const std::string paths[], int count)
{
    for (int index = 0; index < count; ++index)
        std::remove(paths[index].c_str());
}

bool SaveReservation(const WriteReservation &reservation,
                     const FrameSample &sample,
                     std::string &error)
{
    if (sample.frame_bgr.empty() || sample.frame_bgr.type() != CV_8UC3 ||
        sample.frame_bgr.cols != 320 || sample.frame_bgr.rows != 240 ||
        sample.roi80_bgr.empty() || sample.roi80_bgr.type() != CV_8UC3 ||
        sample.roi80_bgr.cols != 80 || sample.roi80_bgr.rows != 80 ||
        sample.model_input64_bgr.empty() || sample.model_input64_bgr.type() != CV_8UC3 ||
        sample.model_input64_bgr.cols != 64 || sample.model_input64_bgr.rows != 64)
    {
        error = "frame contract mismatch; expected 320x240/80x80/64x64 CV_8UC3";
        return false;
    }

    char slot[16] = {};
    std::snprintf(slot, sizeof(slot), "%04d", reservation.slot);
    const std::string names[] = {
        std::string("frame_") + slot + "_bgr.png",
        std::string("frame_") + slot + "_annotated.png",
        std::string("roi80_") + slot + "_bgr.png",
        std::string("model_input64_") + slot + "_bgr.png",
    };
    std::string final_paths[4];
    std::string temporary_paths[4];
    for (int index = 0; index < 4; ++index)
    {
        final_paths[index] = reservation.session_path + "/" + names[index];
        temporary_paths[index] = reservation.session_path + "/tmp_" + names[index];
    }

    cv::Mat annotated = sample.frame_bgr.clone();
    cv::rectangle(annotated, sample.roi_rect, cv::Scalar(0, 255, 0), 1);
    cv::circle(annotated, sample.red_center, 3, cv::Scalar(0, 255, 255), -1);
    const cv::Mat images[] = {
        sample.frame_bgr,
        annotated,
        sample.roi80_bgr,
        sample.model_input64_bgr,
    };
    for (int index = 0; index < 4; ++index)
    {
        if (!cv::imwrite(temporary_paths[index], images[index]))
        {
            error = "image write failed for " + temporary_paths[index];
            RemovePaths(temporary_paths, 4);
            return false;
        }
    }
    for (int index = 0; index < 4; ++index)
    {
        if (!RenameFile(temporary_paths[index], final_paths[index], error))
        {
            RemovePaths(temporary_paths, 4);
            RemovePaths(final_paths, 4);
            return false;
        }
    }

    const std::string manifest_path = reservation.session_path + "/manifest.csv";
    FILE *manifest = std::fopen(manifest_path.c_str(), "a");
    if (manifest == nullptr)
    {
        error = "manifest append failed: " + std::string(std::strerror(errno));
        RemovePaths(final_paths, 4);
        return false;
    }
    const std::uint64_t epoch_ms = EpochMs();
    const int status = std::fprintf(
        manifest,
        "%s,%d,%llu,%u,%s,%s,%d,%d,%d,%d,%d,%d,%.3f,%d,%.6f,%d,%.9f,%.9f,%.9f,%.9f,%ld,%d,%d,%d,%s,%s,%s,%s,%s\n",
        reservation.session_id.c_str(), reservation.slot,
        static_cast<unsigned long long>(epoch_ms), sample.frame_id,
        reservation.business_label.c_str(), reservation.fine_label.c_str(),
        sample.red_center.x, sample.red_center.y,
        sample.roi_rect.x, sample.roi_rect.y,
        sample.roi_rect.width, sample.roi_rect.height,
        sample.distance_cm,
        sample.white_ratio_valid ? 1 : 0,
        sample.white_ratio_valid ? sample.white_ratio : -1.0f,
        sample.class_idx, sample.confidence,
        sample.class_probs[0], sample.class_probs[1], sample.class_probs[2],
        sample.infer_us,
        sample.exposure_config, sample.exposure_mode, sample.exposure_value,
        ExposureOwner(sample.exposure_config),
        names[0].c_str(), names[1].c_str(), names[2].c_str(), names[3].c_str());
    const bool closed = std::fclose(manifest) == 0;
    if (status < 0 || !closed)
    {
        error = "manifest append failed for " + manifest_path;
        RemovePaths(final_paths, 4);
        return false;
    }
    return true;
}

} // namespace

std::string LabelsJson()
{
    std::ostringstream json;
    json << "{\"labels\":[";
    for (std::size_t index = 0; index < sizeof(kLabels) / sizeof(kLabels[0]); ++index)
    {
        if (index != 0)
            json << ',';
        json << "{\"id\":\"" << kLabels[index].fine
             << "\",\"business\":\"" << kLabels[index].business
             << "\",\"display\":\"" << kLabels[index].display << "\"}";
    }
    json << "]}";
    return json.str();
}

std::string StatusJson()
{
    CaptureState &state = State();
    std::lock_guard<std::mutex> lock(state.mutex);
    if (state.active && g_run_mode != RUN_MODE_CAMERA_FOCUS && !state.writing)
    {
        state.active = false;
        state.phase = Phase::kError;
        state.errors++;
        state.last_message = "run mode left camera focus mode";
    }
    return BuildStatusLocked(state, std::chrono::steady_clock::now());
}

bool IsActive()
{
    CaptureState &state = State();
    std::lock_guard<std::mutex> lock(state.mutex);
    ExpireLocked(state, std::chrono::steady_clock::now());
    return state.active;
}

int Start(const std::string &fine_label, const std::string &mode, std::string &response_json)
{
    CaptureState &state = State();
    std::lock_guard<std::mutex> lock(state.mutex);
    const auto now = std::chrono::steady_clock::now();
    ExpireLocked(state, now);
    if (g_run_mode != RUN_MODE_CAMERA_FOCUS)
    {
        response_json = "{\"error\":\"dataset capture requires mode 31\"}";
        return 409;
    }
    if (state.active || state.writing)
    {
        response_json = "{\"error\":\"another capture session is active\"}";
        return 409;
    }
    const LabelDefinition *label = FindLabel(fine_label);
    if (label == nullptr)
    {
        response_json = "{\"error\":\"unknown fine-class label\"}";
        return 400;
    }
    if (mode != "single" && mode != "burst")
    {
        response_json = "{\"error\":\"mode must be single or burst\"}";
        return 400;
    }

    state.generation++;
    state.phase = Phase::kIdle;
    state.active = false;
    state.writing = false;
    state.mode = mode;
    state.fine_label = label->fine;
    state.business_label = label->business;
    state.display_label = label->display;
    state.session_id.clear();
    state.session_path.clear();
    state.last_message.clear();
    state.target = mode == "burst" ? kBurstTarget : kSingleTarget;
    state.scheduled = 0;
    state.saved = 0;
    state.skipped = 0;
    state.errors = 0;
    state.started = now;
    state.next_due = now;
    state.deadline = now + std::chrono::milliseconds(kBurstDurationMs);

    std::string error;
    if (!CreateSessionDirectory(state, *label, error))
    {
        state.phase = Phase::kError;
        state.errors = 1;
        state.last_message = error;
        response_json = BuildStatusLocked(state, now);
        return 500;
    }

    std::ostringstream session;
    session << "{\n"
            << "  \"schema\": 1,\n"
            << "  \"sessionId\": \"" << state.session_id << "\",\n"
            << "  \"businessClass\": \"" << label->business << "\",\n"
            << "  \"fineClass\": \"" << label->fine << "\",\n"
            << "  \"mode\": \"" << mode << "\",\n"
            << "  \"frequencyHz\": " << kBurstHz << ",\n"
            << "  \"durationMs\": " << kBurstDurationMs << ",\n"
            << "  \"target\": " << state.target << ",\n"
            << "  \"createdEpochMs\": " << EpochMs() << ",\n"
            << "  \"groupId\": \"web:" << state.session_id << "\",\n"
            << "  \"frameContract\": \"BGR320x240 + annotated + ROI80 + model_input64\",\n"
            << "  \"modelId\": \"" << LQ_NCNN::ModelId() << "\",\n"
            << "  \"modelParamSha256\": \"" << LQ_NCNN::ExpectedParamSha256() << "\",\n"
            << "  \"modelBinSha256\": \"" << LQ_NCNN::ExpectedBinSha256() << "\"\n"
            << "}\n";
    if (!WriteTextFile(state.session_path + "/session.json", session.str(), error))
    {
        state.phase = Phase::kError;
        state.errors = 1;
        state.last_message = error;
        response_json = BuildStatusLocked(state, now);
        return 500;
    }

    const std::string header =
        "session_id,slot,epoch_ms,frame_id,business_class,fine_class,red_cx,red_cy,"
        "roi_x,roi_y,roi_w,roi_h,distance_cm,white_ratio_valid,white_ratio,class_idx,"
        "confidence,prob_supplies,prob_vehicle,prob_weapon,infer_us,exposure_config,"
        "exposure_mode,exposure_value,exposure_owner,frame_bgr,frame_annotated,roi80_bgr,model_input64_bgr\n";
    if (!WriteTextFile(state.session_path + "/manifest.csv", header, error))
    {
        state.phase = Phase::kError;
        state.errors = 1;
        state.last_message = error;
        response_json = BuildStatusLocked(state, now);
        return 500;
    }

    state.phase = Phase::kCapturing;
    state.active = true;
    state.last_message = "capture started";
    std::printf("[DATASET] start session=%s label=%s/%s mode=%s target=%d path=%s\n",
                state.session_id.c_str(), state.business_label.c_str(),
                state.fine_label.c_str(), state.mode.c_str(), state.target,
                state.session_path.c_str());
    response_json = BuildStatusLocked(state, now);
    return 200;
}

int Cancel(std::string &response_json)
{
    CaptureState &state = State();
    std::lock_guard<std::mutex> lock(state.mutex);
    const auto now = std::chrono::steady_clock::now();
    ExpireLocked(state, now);
    if (!state.active)
    {
        response_json = "{\"error\":\"no active capture session\"}";
        return 409;
    }
    if (state.writing)
    {
        response_json = "{\"error\":\"capture write is in progress; retry cancel\"}";
        return 409;
    }
    const int remaining = std::max(0, state.target - state.scheduled);
    state.scheduled += remaining;
    state.skipped += remaining;
    state.active = false;
    state.phase = Phase::kCancelled;
    state.last_message = "capture cancelled";
    std::printf("[DATASET] cancel session=%s saved=%d skipped=%d\n",
                state.session_id.c_str(), state.saved, state.skipped);
    response_json = BuildStatusLocked(state, now);
    return 200;
}

void OnValidFrame(const FrameSample &sample)
{
    CaptureState &state = State();
    WriteReservation reservation;
    {
        std::lock_guard<std::mutex> lock(state.mutex);
        const auto now = std::chrono::steady_clock::now();
        ExpireLocked(state, now);
        if (!state.active || state.writing)
            return;
        if (g_run_mode != RUN_MODE_CAMERA_FOCUS)
        {
            state.active = false;
            state.phase = Phase::kError;
            state.errors++;
            state.last_message = "capture frame received outside mode 31";
            return;
        }
        if (now < state.next_due)
            return;

        while (state.scheduled < state.target &&
               state.next_due + std::chrono::milliseconds(kBurstPeriodMs) <= now)
        {
            state.scheduled++;
            state.skipped++;
            state.next_due += std::chrono::milliseconds(kBurstPeriodMs);
        }
        if (state.scheduled >= state.target)
        {
            state.active = false;
            state.phase = Phase::kCompleted;
            state.last_message = "capture completed";
            return;
        }

        reservation.generation = state.generation;
        reservation.slot = state.scheduled;
        reservation.session_id = state.session_id;
        reservation.session_path = state.session_path;
        reservation.mode = state.mode;
        reservation.fine_label = state.fine_label;
        reservation.business_label = state.business_label;
        state.scheduled++;
        state.next_due += std::chrono::milliseconds(kBurstPeriodMs);
        state.writing = true;
    }

    std::string error;
    const bool saved = SaveReservation(reservation, sample, error);

    std::lock_guard<std::mutex> lock(state.mutex);
    if (state.generation != reservation.generation)
        return;
    state.writing = false;
    if (!saved)
    {
        state.errors++;
        state.active = false;
        state.phase = Phase::kError;
        state.last_message = error;
        std::printf("[DATASET] error session=%s slot=%d error=%s\n",
                    reservation.session_id.c_str(), reservation.slot, error.c_str());
        return;
    }
    state.saved++;
    state.last_message = "saved slot " + std::to_string(reservation.slot);
    std::printf("[DATASET] saved session=%s slot=%d saved=%d/%d frame=%u\n",
                reservation.session_id.c_str(), reservation.slot,
                state.saved, state.target, sample.frame_id);
    if (state.scheduled >= state.target)
    {
        state.active = false;
        state.phase = Phase::kCompleted;
        state.last_message = "capture completed";
        std::printf("[DATASET] complete session=%s saved=%d skipped=%d errors=%d\n",
                    state.session_id.c_str(), state.saved, state.skipped, state.errors);
    }
}

} // namespace dataset_capture
