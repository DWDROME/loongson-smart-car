#include "race_recognition_result.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <unistd.h>

namespace race_recognition_result
{
namespace
{

constexpr const char *kResultPath = "/root/race_recognition_result.log";

bool CopyText(char *destination, std::size_t destination_size, const char *source)
{
    if (destination == nullptr || destination_size == 0 || source == nullptr)
    {
        return false;
    }
    const int written = std::snprintf(destination, destination_size, "%s", source);
    return written >= 0 && static_cast<std::size_t>(written) < destination_size;
}

Recorder g_recorder(kResultPath);

} // namespace

const char *ToString(RecognitionClass value)
{
    switch (value)
    {
    case RecognitionClass::kSupplies:
        return "supplies";
    case RecognitionClass::kVehicle:
        return "vehicle";
    case RecognitionClass::kWeapon:
        return "weapon";
    default:
        return nullptr;
    }
}

Recorder::Recorder(const char *result_path)
{
    if (!CopyText(result_path_, sizeof(result_path_), result_path))
    {
        result_path_[0] = '\0';
    }
}

bool Recorder::Reset(const char *model_id)
{
    count_ = 0;
    overflowed_ = false;
    finished_ = false;
    initialized_ = false;

    if (result_path_[0] == '\0')
    {
        std::printf("[RACE_RESULT] invalid result path\n");
        return false;
    }
    if (!CopyText(model_id_, sizeof(model_id_), model_id))
    {
        std::printf("[RACE_RESULT] invalid or too-long model id\n");
        return false;
    }

    initialized_ = true;
    if (!WriteSnapshot(false))
    {
        initialized_ = false;
        return false;
    }
    std::printf("[RACE_RESULT] reset model=%s capacity=%zu path=%s\n",
                model_id_, kCapacity, result_path_);
    return true;
}

bool Recorder::Record(RecognitionClass value)
{
    const char *label = ToString(value);
    if (!initialized_)
    {
        std::printf("[RACE_RESULT] record rejected: recorder not initialized\n");
        return false;
    }
    if (finished_)
    {
        std::printf("[RACE_RESULT] record rejected: race already finished\n");
        return false;
    }
    if (label == nullptr)
    {
        std::printf("[RACE_RESULT] record rejected: unknown class\n");
        return false;
    }
    if (count_ >= kCapacity)
    {
        overflowed_ = true;
        if (!WriteSnapshot(false))
        {
            std::printf("[RACE_RESULT] overflow flag write failed\n");
        }
        std::printf("[RACE_RESULT] overflow: capacity=%zu class=%s\n", kCapacity, label);
        return false;
    }

    results_[count_] = value;
    count_++;
    if (!WriteSnapshot(false))
    {
        return false;
    }
    std::printf("[RACE_RESULT] event=%zu class=%s\n", count_, label);
    return true;
}

bool Recorder::Finalize()
{
    if (!initialized_)
    {
        std::printf("[RACE_RESULT] finalize rejected: recorder not initialized\n");
        return false;
    }
    if (finished_)
    {
        return true;
    }

    finished_ = true;
    if (!WriteSnapshot(true))
    {
        finished_ = false;
        return false;
    }
    std::printf("[RACE_RESULT] finished count=%zu overflow=%u path=%s\n",
                count_, overflowed_ ? 1u : 0u, result_path_);
    return true;
}

bool Recorder::WriteSnapshot(bool durable)
{
    char temporary_path[192] = {};
    const int path_length = std::snprintf(
        temporary_path, sizeof(temporary_path), "%s.tmp.%ld",
        result_path_, static_cast<long>(getpid()));
    if (path_length < 0 || static_cast<std::size_t>(path_length) >= sizeof(temporary_path))
    {
        std::printf("[RACE_RESULT] temporary path too long\n");
        return false;
    }

    FILE *file = std::fopen(temporary_path, "w");
    if (file == nullptr)
    {
        std::printf("[RACE_RESULT] open failed path=%s error=%s\n",
                    temporary_path, std::strerror(errno));
        return false;
    }

    bool ok = true;
    ok = ok && std::fprintf(file, "version=1\n") >= 0;
    ok = ok && std::fprintf(file, "status=%s\n", finished_ ? "finished" : "running") >= 0;
    ok = ok && std::fprintf(file, "model_id=%s\n", model_id_) >= 0;
    ok = ok && std::fprintf(file, "count=%zu\n", count_) >= 0;
    ok = ok && std::fprintf(file, "overflow=%u\n", overflowed_ ? 1u : 0u) >= 0;
    ok = ok && std::fprintf(file, "sequence=") >= 0;
    for (std::size_t index = 0; ok && index < count_; ++index)
    {
        const char *label = ToString(results_[index]);
        if (label == nullptr)
        {
            ok = false;
            break;
        }
        if (index != 0)
        {
            ok = std::fputc(',', file) != EOF;
        }
        if (ok)
        {
            ok = std::fputs(label, file) != EOF;
        }
    }
    if (ok)
    {
        ok = std::fputc('\n', file) != EOF;
    }
    if (ok)
    {
        ok = std::fflush(file) == 0;
    }
    if (ok && durable)
    {
        ok = fsync(fileno(file)) == 0;
    }
    if (std::fclose(file) != 0)
    {
        ok = false;
    }

    if (!ok)
    {
        const int saved_errno = errno;
        unlink(temporary_path);
        std::printf("[RACE_RESULT] write failed path=%s error=%s\n",
                    temporary_path, std::strerror(saved_errno));
        return false;
    }
    if (std::rename(temporary_path, result_path_) != 0)
    {
        const int saved_errno = errno;
        unlink(temporary_path);
        std::printf("[RACE_RESULT] rename failed path=%s error=%s\n",
                    result_path_, std::strerror(saved_errno));
        return false;
    }
    return true;
}

bool Reset(const char *model_id)
{
    return g_recorder.Reset(model_id);
}

bool Record(RecognitionClass value)
{
    return g_recorder.Record(value);
}

bool Finalize()
{
    return g_recorder.Finalize();
}

const char *ResultPath()
{
    return kResultPath;
}

} // namespace race_recognition_result
