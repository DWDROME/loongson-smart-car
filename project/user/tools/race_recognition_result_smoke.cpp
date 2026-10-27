#include "race_recognition_result.hpp"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>

namespace
{

void Require(bool condition, const char *message)
{
    if (!condition)
    {
        std::fprintf(stderr, "FAIL: %s\n", message);
        std::exit(1);
    }
}

std::string ReadAll(const char *path)
{
    std::ifstream input(path);
    std::ostringstream contents;
    contents << input.rdbuf();
    return contents.str();
}

bool Contains(const std::string &text, const char *needle)
{
    return text.find(needle) != std::string::npos;
}

} // namespace

int main()
{
    constexpr const char *kPath = "/tmp/race_recognition_result_smoke.log";
    std::remove(kPath);

    race_recognition_result::Recorder recorder(kPath);
    Require(recorder.Reset("smoke_model"), "reset");
    Require(recorder.Record(race_recognition_result::RecognitionClass::kSupplies),
            "record supplies");
    Require(recorder.Record(race_recognition_result::RecognitionClass::kVehicle),
            "record vehicle 1");
    Require(recorder.Record(race_recognition_result::RecognitionClass::kVehicle),
            "record vehicle 2");
    Require(recorder.Record(race_recognition_result::RecognitionClass::kWeapon),
            "record weapon");
    Require(recorder.Record(race_recognition_result::RecognitionClass::kSupplies),
            "record supplies 2");
    Require(recorder.Finalize(), "finalize");
    Require(recorder.Finalize(), "finalize idempotent");

    std::string result = ReadAll(kPath);
    Require(Contains(result, "status=finished\n"), "finished status");
    Require(Contains(result, "model_id=smoke_model\n"), "model id");
    Require(Contains(result, "count=5\n"), "five events");
    Require(Contains(result, "overflow=0\n"), "no overflow");
    Require(Contains(result, "sequence=supplies,vehicle,vehicle,weapon,supplies\n"),
            "ordered sequence");

    Require(recorder.Reset("overflow_model"), "overflow reset");
    for (std::size_t index = 0; index < race_recognition_result::kCapacity; ++index)
    {
        Require(recorder.Record(race_recognition_result::RecognitionClass::kWeapon),
                "fill capacity");
    }
    Require(!recorder.Record(race_recognition_result::RecognitionClass::kVehicle),
            "ninth event rejected");
    Require(recorder.Finalize(), "overflow finalize");

    result = ReadAll(kPath);
    Require(Contains(result, "count=8\n"), "capacity count");
    Require(Contains(result, "overflow=1\n"), "overflow visible");
    Require(Contains(result,
                     "sequence=weapon,weapon,weapon,weapon,weapon,weapon,weapon,weapon\n"),
            "capacity sequence preserved");

    std::remove(kPath);
    std::puts("race_recognition_result_smoke: passed");
    return 0;
}
