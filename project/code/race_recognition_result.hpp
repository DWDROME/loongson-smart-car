#ifndef RACE_RECOGNITION_RESULT_HPP
#define RACE_RECOGNITION_RESULT_HPP

#include <cstddef>
#include <cstdint>

namespace race_recognition_result
{

enum class RecognitionClass : std::uint8_t
{
    kSupplies = 0,
    kVehicle = 1,
    kWeapon = 2,
};

constexpr std::size_t kCapacity = 8;

const char *ToString(RecognitionClass value);

class Recorder
{
public:
    explicit Recorder(const char *result_path);

    bool Reset(const char *model_id);
    bool Record(RecognitionClass value);
    bool Finalize();

    std::size_t count() const { return count_; }
    bool overflowed() const { return overflowed_; }
    bool finished() const { return finished_; }

private:
    bool WriteSnapshot(bool durable);

    char result_path_[160] = {};
    char model_id_[128] = {};
    RecognitionClass results_[kCapacity] = {};
    std::size_t count_ = 0;
    bool overflowed_ = false;
    bool finished_ = false;
    bool initialized_ = false;
};

bool Reset(const char *model_id);
bool Record(RecognitionClass value);
bool Finalize();
const char *ResultPath();

} // namespace race_recognition_result

#endif
