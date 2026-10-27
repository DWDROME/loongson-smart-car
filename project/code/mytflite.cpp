#include "zf_common_headfile.hpp"
#include "mytflite.h"
#include "loong_cnn_model_simple.h"
#include <sys/mman.h>
#include <new>

// ======================== 可配置参数宏定义 ========================
#define MODEL_INPUT_WIDTH 60
#define MODEL_INPUT_HEIGHT 60
#define MODEL_INPUT_CHANNEL 3
#define MODEL_OUTPUT_CLASS_NUM 3
#define MODEL_INPUT_SIZE (MODEL_INPUT_WIDTH * MODEL_INPUT_HEIGHT * MODEL_INPUT_CHANNEL)
// LQ w0.6：DW/PW + Mean/FC/Softmax；BN/rescale 常拆成 Mul/Add/Div
#define TFLITE_OP_RESOLVER_MAX_NUM 16
#define TENSOR_ARENA_SIZE (512 * 1024)

namespace
{
    constexpr const char *kModelId =
        "lq_w0.6_mt_ls0.1_20260713 sha256="
        "6702e88f962a9989bd246633c248fd09c1139cf5de7d80235bbca17e1196c855";
    // Current-camera photometric alignment for the LQ classifier only.
    constexpr float kModelInputGain = 2.2f;
    const char *class_labels[] = {"materials", "traffic", "weapon"};

    bool g_tflite_inited = false;
    // bool g_first_frame_logged = false;

    const tflite::Model *g_model = nullptr;
    // uint8_t g_tensor_arena[TENSOR_ARENA_SIZE];

    uint8_t *g_model_raw = nullptr;
    uint8_t *g_model_buf = nullptr;

    uint8_t *g_arena_raw = nullptr;
    uint8_t *g_tensor_arena = nullptr;
    tflite::MicroInterpreter *g_interpreter = nullptr;

    alignas(alignof(tflite::MicroInterpreter)) uint8_t g_interpreter_buf[sizeof(tflite::MicroInterpreter)];

    void *alloc_low_mem(size_t size, size_t alignment)
    {
        const size_t page_size = (size_t)sysconf(_SC_PAGESIZE);
        const size_t alloc_size = (size + page_size - 1) & ~(page_size - 1);

#ifdef MAP_FIXED_NOREPLACE
        // 优先尝试在 4GB 以下的固定低地址区间分配，避免高地址被错误截断
        for (uintptr_t base = 0x10000000u; base < 0x70000000u; base += 0x01000000u)
        {
            void *ptr = mmap(reinterpret_cast<void *>(base), alloc_size,
                             PROT_READ | PROT_WRITE,
                             MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE,
                             -1, 0);
            if (ptr != MAP_FAILED)
            {
                if (alignment > 0 && (reinterpret_cast<uintptr_t>(ptr) % alignment) != 0)
                {
                    munmap(ptr, alloc_size);
                    continue;
                }
                return ptr;
            }
        }
#endif

        // 先尝试 mmap 到 4GB 以内地址
        const uintptr_t max_addr = 0x100000000ULL;
        for (int i = 0; i < 32; ++i)
        {
            uintptr_t hint = 0x10000000ULL + (uintptr_t)i * 0x01000000ULL;
            void *ptr = mmap(reinterpret_cast<void *>(hint), size, PROT_READ | PROT_WRITE,
                             MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
            if (ptr != MAP_FAILED)
            {
                uintptr_t addr = reinterpret_cast<uintptr_t>(ptr);
                if (addr < max_addr && (alignment == 0 || (addr % alignment) == 0))
                {
                    return ptr;
                }
                munmap(ptr, size);
            }
        }

        // 兜底：常规分配
        void *ptr = nullptr;
        if (posix_memalign(&ptr, alignment, size) == 0)
        {
            return ptr;
        }
        return nullptr;
    }

    void debug_log_printf_callback(const char *s)
    {
        if (NULL == s)
            return;
        printf("%s", s);
    }
}

void tflite_init(void)
{
    if (g_tflite_inited)
    {
        return;
    }

    // 重定向TFLM日志（显式取本文件匿名命名空间内回调，避免与 image 侧同名冲突）
    RegisterDebugLogCallback(+[](const char *s) {
        if (s)
            printf("%s", s);
    });

    // tflite库初始化
    tflite::InitializeTarget();

    // 导入模型（拷贝到低地址RAM，避免地址截断/映射异常）
    const size_t model_align = 16;
    const size_t model_alloc = loong_cnn_model_simple_tflite_len + model_align;
    g_model_raw = static_cast<uint8_t *>(alloc_low_mem(model_alloc, model_align));
    if (g_model_raw != nullptr)
    {
        uintptr_t raw_addr = reinterpret_cast<uintptr_t>(g_model_raw);
        uintptr_t aligned = (raw_addr + (model_align - 1)) & ~(uintptr_t)(model_align - 1);
        g_model_buf = reinterpret_cast<uint8_t *>(aligned);
    }
    else
    {
        g_model_buf = nullptr;
    }
    if (g_model_buf == nullptr)
    {
        printf("❌ TFLite model buffer 分配失败\n");
        return;
    }
    memcpy(g_model_buf, loong_cnn_model_simple_tflite, loong_cnn_model_simple_tflite_len);
    g_model = ::tflite::GetModel(g_model_buf);
    if (g_model == nullptr || g_model->version() != TFLITE_SCHEMA_VERSION)
    {
        printf("TFLite schema mismatch: model=%d runtime=%d\n",
               g_model == nullptr ? -1 : g_model->version(), TFLITE_SCHEMA_VERSION);
        return;
    }

    // LQ w0.6 (export_lq PureLqBusiness): Conv + Depthwise + BN/ReLU 图 + GAP Mean + Dense Softmax
    // 注意：本板 libtflm 对 Sub/Rsqrt 链接不完整，勿注册
    static tflite::MicroMutableOpResolver<TFLITE_OP_RESOLVER_MAX_NUM> resolver;
    resolver.AddConv2D();
    resolver.AddDepthwiseConv2D();
    resolver.AddFullyConnected();
    resolver.AddSoftmax();
    resolver.AddMean();
    resolver.AddAveragePool2D();
    resolver.AddMaxPool2D();
    resolver.AddRelu();
    resolver.AddRelu6();
    resolver.AddMul();
    resolver.AddAdd();
    resolver.AddDiv();
    resolver.AddReshape();
    resolver.AddSqueeze();
    resolver.AddPad();
    resolver.AddQuantize();

    // 分配 arena
    const size_t arena_align = 32;
    const size_t arena_alloc = TENSOR_ARENA_SIZE + arena_align;
    g_arena_raw = static_cast<uint8_t *>(alloc_low_mem(arena_alloc, arena_align));
    if (g_arena_raw != nullptr)
    {
        uintptr_t raw_addr = reinterpret_cast<uintptr_t>(g_arena_raw);
        uintptr_t aligned = (raw_addr + (arena_align - 1)) & ~(uintptr_t)(arena_align - 1);
        g_tensor_arena = reinterpret_cast<uint8_t *>(aligned);
    }
    else
    {
        g_tensor_arena = nullptr;
    }
    if (g_tensor_arena == nullptr)
    {
        printf("❌ TFLite arena 分配失败\n");
        return;
    }

    // 创建解释器（placement new）
    g_interpreter = new (g_interpreter_buf) tflite::MicroInterpreter(g_model, resolver, g_tensor_arena, TENSOR_ARENA_SIZE);
    if (g_interpreter->AllocateTensors() != kTfLiteOk)
    {
        g_tflite_inited = false;
        printf("❌ TFLite AllocateTensors 失败\n");
        return;
    }

    const TfLiteTensor *input_tensor = g_interpreter->input(0);
    const TfLiteTensor *output_tensor = g_interpreter->output(0);
    if (input_tensor == nullptr || input_tensor->type != kTfLiteFloat32 ||
        input_tensor->dims == nullptr || input_tensor->dims->size != 4 ||
        input_tensor->dims->data[0] != 1 ||
        input_tensor->dims->data[1] != MODEL_INPUT_HEIGHT ||
        input_tensor->dims->data[2] != MODEL_INPUT_WIDTH ||
        input_tensor->dims->data[3] != MODEL_INPUT_CHANNEL ||
        input_tensor->bytes != MODEL_INPUT_SIZE * sizeof(float) ||
        output_tensor == nullptr || output_tensor->type != kTfLiteFloat32 ||
        output_tensor->dims == nullptr || output_tensor->dims->size != 2 ||
        output_tensor->dims->data[0] != 1 ||
        output_tensor->dims->data[1] != MODEL_OUTPUT_CLASS_NUM ||
        output_tensor->bytes != MODEL_OUTPUT_CLASS_NUM * sizeof(float))
    {
        printf("TFLite tensor contract mismatch\n");
        return;
    }

    g_tflite_inited = true;
    printf("TFLite initialized: %s input=1x60x60x3 output=1x3 input_gain=%.1f\n",
           kModelId, kModelInputGain);
}

const char *tflite_get_label(int class_idx)
{
    if (class_idx < 0 || class_idx >= MODEL_OUTPUT_CLASS_NUM)
    {
        return "unknown";
    }
    return class_labels[class_idx];
}

bool tflite_run(Mat img, int &class_idx, float &confidence, long &infer_us,
                float class_probs[3])
{
    class_idx = -1;    // 输出类别索引
    confidence = 0.0f; // 置信度
    infer_us = 0;

    if (!g_tflite_inited || NULL == g_interpreter)
    {
        return false;
    }

    if (img.empty() || img.type() != CV_8UC3 ||
        img.cols != MODEL_INPUT_WIDTH || img.rows != MODEL_INPUT_HEIGHT)
    {
        return false;
    }

    // ======================== 图像预处理 ========================
    Mat float_img;

    // 输入合同：60x60 BGR uint8 -> BGR float32 * gain -> clamp [0,255] -> RGB；模型内部完成 /255。
    img.convertTo(float_img, CV_32FC3, kModelInputGain);
    cv::min(float_img, cv::Scalar::all(255.0f), float_img);

    // BGR -> RGB
    cvtColor(float_img, float_img, cv::COLOR_BGR2RGB);

    // 确保连续内存
    Mat continuous_float_img = float_img.isContinuous() ? float_img : float_img.clone();
    if (continuous_float_img.total() * continuous_float_img.elemSize() != (MODEL_INPUT_SIZE * sizeof(float)))
    {
        return false;
    }
    float *model_input_ptr = continuous_float_img.ptr<float>(0);

    // // 第一帧调试日志
    // if (ENABLE_FIRST_FRAME_DEBUG && !g_first_frame_logged)
    // {
    //     printf("\n===== 第1帧 完整预处理日志 =====\n");
    //     printf("1. 原始图像：%d x %d (宽x高)，通道数：%d\n", img.cols, img.rows, img.channels());
    //     // printf("2. 缩放后图像：%d x %d (宽x高)，通道数：%d\n", resized_img.cols, resized_img.rows, resized_img.channels());
    //     printf("3. 浮点图像内存连续：%s\n", float_img.isContinuous() ? "是" : "否");
    //     printf("4. 模型输入尺寸匹配：%s\n",
    //            (continuous_float_img.total() * continuous_float_img.elemSize()) == (MODEL_INPUT_SIZE * sizeof(float))
    //                ? "✅ 匹配"
    //                : "❌ 不匹配");
    //     printf("=================================\n");
    //     g_first_frame_logged = true;
    // }

    // ======================== 模型输入 ========================
    TfLiteTensor *input_tensor = g_interpreter->input(0);
    if (nullptr == input_tensor)
    {
        return false;
    }
    if (input_tensor->type != kTfLiteFloat32)
    {
        return false;
    }
    if (input_tensor->dims == nullptr || input_tensor->dims->size != 4)
    {
        return false;
    }
    if (input_tensor->dims->data[0] != 1 ||
        input_tensor->dims->data[1] != MODEL_INPUT_HEIGHT ||
        input_tensor->dims->data[2] != MODEL_INPUT_WIDTH ||
        input_tensor->dims->data[3] != MODEL_INPUT_CHANNEL)
    {
        return false;
    }
    if (input_tensor->bytes != (MODEL_INPUT_SIZE * sizeof(float)))
    {
        return false;
    }
    float *input_data = input_tensor->data.f;
    if (input_data == nullptr)
    {
        return false;
    }
    memcpy(input_data, model_input_ptr, MODEL_INPUT_SIZE * sizeof(float));

    // ======================== 模型推理 ========================
    auto start_time1 = std::chrono::high_resolution_clock::now();
    const TfLiteStatus invoke_status = g_interpreter->Invoke();
    auto end_time1 = std::chrono::high_resolution_clock::now();
    infer_us = std::chrono::duration_cast<std::chrono::microseconds>(end_time1 - start_time1).count();
    if (invoke_status != kTfLiteOk)
    {
        printf("TFLite Invoke failed\n");
        return false;
    }

    // ======================== 模型输出解析 ========================
    TfLiteTensor *output_tensor = g_interpreter->output(0);
    if (output_tensor == nullptr || output_tensor->type != kTfLiteFloat32 ||
        output_tensor->dims == nullptr || output_tensor->dims->size != 2 ||
        output_tensor->dims->data[0] != 1 ||
        output_tensor->dims->data[1] != MODEL_OUTPUT_CLASS_NUM ||
        output_tensor->bytes != MODEL_OUTPUT_CLASS_NUM * sizeof(float) ||
        output_tensor->data.f == nullptr)
    {
        return false;
    }
    float *output_data = output_tensor->data.f;
    if (class_probs != nullptr)
    {
        for (int n = 0; n < MODEL_OUTPUT_CLASS_NUM; n++)
        {
            class_probs[n] = output_data[n];
        }
    }
    float max_v = output_data[0];
    int max_i = 0;
    for (int n = 1; n < MODEL_OUTPUT_CLASS_NUM; n++)
    {
        float v = output_data[n];
        if (v > max_v)
        {
            max_v = v;
            max_i = n;
        }
    }

    class_idx = max_i;
    confidence = max_v;
    return true;
}
